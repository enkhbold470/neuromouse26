"""MM26 supervisor — autonomous run controller with camera-aided wall learning.

Loop:
    1. (Optional) push learned wall consensus to firmware → /cmd/start → /cmd/walls
    2. Watch /state at SUPER_HZ. Accumulate camera-observed walls each tick.
    3. Detect terminal state:
         - GOAL  → success; bump consensus weight; log episode
         - CRASH → fail; /cmd/clear_crash; log episode
         - timeout / no-progress → /cmd/stop; log episode
    4. Loop until --episodes exhausted.

Mistakes caught:
    * pose_error_mm — encoder drift / ticks_per_cell wrong / RIGHT_ENC_SCALE off
    * pose_error_deg — gyro bias / turn convergence
    * wall_false_pos — firmware believes wall where camera sees none
    * wall_false_neg — firmware misses real wall (most dangerous → crash)

Outputs (in tools/vision/runs/):
    episode_<ts>.jsonl — per-tick samples for that run
    summary.csv        — one row per episode
    walls_consensus.json — running vote tally + threshold-decided walls
"""

import argparse
import json
import os
import sys
import time
from collections import deque

import requests

ROWS = 6
COLS = 3
DIR_NAMES = ["N", "E", "S", "W"]
WALL_BITS = [1, 2, 4, 8]   # N,E,S,W

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RUNS_DIR = os.path.join(SCRIPT_DIR, "runs")
WALLS_CONSENSUS = os.path.join(RUNS_DIR, "walls_consensus.json")
SUMMARY_CSV = os.path.join(RUNS_DIR, "summary.csv")


def ensure_dirs():
    os.makedirs(RUNS_DIR, exist_ok=True)


def http_get(url, timeout=1.0):
    try:
        r = requests.get(url, timeout=timeout)
        r.raise_for_status()
        return r.json() if "json" in r.headers.get("Content-Type", "") else r.text
    except Exception as e:
        return {"_error": str(e)}


def cmd(fw_url, path, **params):
    url = fw_url.rstrip("/") + "/cmd/" + path
    try:
        r = requests.get(url, params=params, timeout=1.5)
        return r.status_code == 200, r.text
    except Exception as e:
        return False, str(e)


def load_consensus():
    if os.path.exists(WALLS_CONSENSUS):
        with open(WALLS_CONSENSUS) as f:
            d = json.load(f)
        return d
    return {
        "present": [[[0] * 4 for _ in range(COLS)] for _ in range(ROWS)],
        "total":   [[[0] * 4 for _ in range(COLS)] for _ in range(ROWS)],
        "episodes": 0,
    }


def save_consensus(c):
    tmp = WALLS_CONSENSUS + ".tmp"
    with open(tmp, "w") as f:
        json.dump(c, f, indent=2)
    os.replace(tmp, WALLS_CONSENSUS)


def accumulate(consensus, cam_walls):
    """Each frame is a vote. cam_walls[r][c] is 4-bit mask."""
    for r in range(ROWS):
        for c in range(COLS):
            m = cam_walls[r][c]
            for i, b in enumerate(WALL_BITS):
                consensus["total"][r][c][i] += 1
                if m & b:
                    consensus["present"][r][c][i] += 1


def decide_walls(consensus, min_votes=15, present_frac=0.55):
    """Build walls[6][3] mask using vote ratio. Skips low-confidence edges."""
    walls = [[0] * COLS for _ in range(ROWS)]
    confident = [[0] * COLS for _ in range(ROWS)]
    for r in range(ROWS):
        for c in range(COLS):
            for i, b in enumerate(WALL_BITS):
                t = consensus["total"][r][c][i]
                p = consensus["present"][r][c][i]
                if t < min_votes:
                    continue
                confident[r][c] |= b
                if p / t >= present_frac:
                    walls[r][c] |= b
    return walls, confident


def walls_to_hex(walls):
    out = []
    for r in range(ROWS):
        for c in range(COLS):
            out.append(f"{walls[r][c] & 0xF:02x}")
    return "".join(out)


def diff_walls(a, b):
    n = 0
    if a is None or b is None:
        return None
    for r in range(ROWS):
        for c in range(COLS):
            x = (int(a[r][c]) ^ int(b[r][c])) & 0xF
            while x:
                n += x & 1
                x >>= 1
    return n


def append_summary_row(row):
    new = not os.path.exists(SUMMARY_CSV)
    with open(SUMMARY_CSV, "a") as f:
        if new:
            f.write(",".join(row.keys()) + "\n")
        f.write(",".join(str(v) for v in row.values()) + "\n")


def fmt_walls(walls):
    lines = []
    for r in range(ROWS - 1, -1, -1):
        line = ""
        for c in range(COLS):
            m = walls[r][c]
            s = ""
            s += "N" if m & 1 else "."
            s += "E" if m & 2 else "."
            s += "S" if m & 4 else "."
            s += "W" if m & 8 else "."
            line += s + " "
        lines.append(line)
    return "\n".join(lines)


def run_episode(args, ep_idx, consensus):
    fw_url = args.firmware
    cam_url = args.cam
    ts = int(time.time())
    log_path = os.path.join(RUNS_DIR, f"episode_{ts}.jsonl")
    log_f = open(log_path, "w")

    # Pre-run: push consensus walls if we have any confident edges and learning
    # is enabled. Sequence: /cmd/reset → wait IDLE → /cmd/start → /cmd/walls.
    cmd(fw_url, "reset")
    time.sleep(0.4)
    push_walls = None
    if args.learn and consensus["episodes"] >= args.min_eps_to_push:
        walls, confident = decide_walls(consensus, args.min_votes, args.present_frac)
        if any(any(row) for row in confident):
            push_walls = walls
            print(f"[ep {ep_idx}] pushing learned walls "
                  f"(after {consensus['episodes']} episodes)")
            print(fmt_walls(walls))

    ok, _ = cmd(fw_url, "start")
    if not ok:
        print(f"[ep {ep_idx}] start failed; aborting")
        log_f.close()
        return None

    if push_walls is not None:
        # Give the firmware a tick to process /cmd/start so setupMaze finishes
        # before our walls land (applyCmds processes start first then walls).
        time.sleep(0.05)
        cmd(fw_url, "walls", w=walls_to_hex(push_walls))

    # Watch loop.
    t0 = time.time()
    last_progress = t0
    last_cell = None
    samples = 0
    crashed = False
    goaled = False
    timed_out = False
    pose_drift_max_mm = 0.0
    cell_mismatches = 0
    wall_mismatch_sum = 0
    visited_cells = set()

    while True:
        now = time.time()
        if now - t0 > args.max_run_s:
            timed_out = True
            cmd(fw_url, "stop")
            break

        # camera snapshot (has both cam+fw merged)
        m = http_get(cam_url, timeout=0.6)
        if isinstance(m, dict) and "_error" not in m:
            cam_walls = m.get("camera", {}).get("walls")
            cam_robot = m.get("camera", {}).get("robot") or {}
            fw = m.get("firmware") or {}
            diff = m.get("diff") or {}

            if cam_walls:
                accumulate(consensus, cam_walls)

            # Pose drift tracking.
            pe = diff.get("pose_error_mm")
            if pe is not None:
                pose_drift_max_mm = max(pose_drift_max_mm, pe)
            if diff.get("cell_match") is False:
                cell_mismatches += 1
            wm = diff.get("wall_mismatches")
            if wm is not None:
                wall_mismatch_sum += wm

            state = fw.get("state")
            crash = fw.get("crash")
            row, col = fw.get("row"), fw.get("col")
            if row is not None and col is not None:
                if last_cell != (row, col):
                    last_cell = (row, col)
                    last_progress = now
                    visited_cells.add((row, col))

            # Log every tick.
            log_f.write(json.dumps({
                "t": now - t0,
                "cam_r":   cam_robot.get("row"),
                "cam_c":   cam_robot.get("col"),
                "cam_h":   cam_robot.get("heading_dir"),
                "cam_xmm": cam_robot.get("x_mm"),
                "cam_ymm": cam_robot.get("y_mm"),
                "cam_det": cam_robot.get("detected", False),
                "fw_r":    row, "fw_c": col, "fw_h": fw.get("heading"),
                "fw_state": state, "fw_phase": fw.get("phase"),
                "fw_crash": crash,
                "diff_pe_mm": pe, "diff_de": diff.get("pose_error_deg"),
                "diff_cell_match": diff.get("cell_match"),
                "diff_wall_mis": wm,
            }) + "\n")
            samples += 1

            if state == "GOAL":
                goaled = True
                break
            if state == "CRASH" or crash:
                crashed = True
                cmd(fw_url, "clear_crash")
                break

            if now - last_progress > args.no_progress_s:
                print(f"[ep {ep_idx}] no progress {args.no_progress_s}s; stopping")
                timed_out = True
                cmd(fw_url, "stop")
                break

        time.sleep(1.0 / args.super_hz)

    log_f.close()
    consensus["episodes"] += 1
    save_consensus(consensus)

    elapsed = time.time() - t0
    result = "GOAL" if goaled else ("CRASH" if crashed else "TIMEOUT")
    row = {
        "ts": ts,
        "ep": ep_idx,
        "result": result,
        "elapsed_s": round(elapsed, 1),
        "samples": samples,
        "visited": len(visited_cells),
        "cell_mismatches": cell_mismatches,
        "wall_mismatch_sum": wall_mismatch_sum,
        "pose_drift_max_mm": round(pose_drift_max_mm, 1),
        "pushed_walls": int(push_walls is not None),
    }
    append_summary_row(row)
    print(f"[ep {ep_idx}] {result} in {elapsed:.1f}s  "
          f"visited={len(visited_cells)}  cell_mis={cell_mismatches}  "
          f"wall_mis={wall_mismatch_sum}  pose_drift_max={pose_drift_max_mm:.1f}mm")
    return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--firmware", default="http://mm26.local",
                    help="firmware base URL (no trailing /state)")
    ap.add_argument("--cam", default="http://localhost:5005/map.json",
                    help="camera JSON URL")
    ap.add_argument("--episodes", type=int, default=10)
    ap.add_argument("--super-hz", type=float, default=10.0)
    ap.add_argument("--max-run-s", type=float, default=120.0)
    ap.add_argument("--no-progress-s", type=float, default=20.0,
                    help="abort if firmware cell doesn't advance for this long")
    ap.add_argument("--learn", action="store_true",
                    help="push wall consensus to firmware before /cmd/start")
    ap.add_argument("--min-eps-to-push", type=int, default=2)
    ap.add_argument("--min-votes", type=int, default=15)
    ap.add_argument("--present-frac", type=float, default=0.55)
    ap.add_argument("--inter-ep-s", type=float, default=2.0)
    args = ap.parse_args()

    if "/state" in args.firmware or "/cmd" in args.firmware:
        sys.exit("--firmware must be the base URL (e.g. http://mm26.local), not /state.")

    ensure_dirs()
    consensus = load_consensus()
    print(f"Supervisor: fw={args.firmware}  cam={args.cam}  "
          f"episodes={args.episodes}  learn={args.learn}")
    print(f"Loaded consensus: {consensus['episodes']} prior episodes")

    for ep in range(args.episodes):
        try:
            run_episode(args, ep, consensus)
        except KeyboardInterrupt:
            print("\nInterrupted. Stopping robot.")
            cmd(args.firmware, "stop")
            break
        if ep + 1 < args.episodes:
            time.sleep(args.inter_ep_s)

    walls, confident = decide_walls(consensus, args.min_votes, args.present_frac)
    print("\n=== Final wall consensus (after",
          consensus["episodes"], "episodes) ===")
    print(fmt_walls(walls))
    print(f"hex: {walls_to_hex(walls)}")
    print(f"summary CSV: {SUMMARY_CSV}")


if __name__ == "__main__":
    main()

"""Top-down USB-camera maze + robot tracker for the 6x3 micromouse arena.

Purpose: ground-truth comparison against firmware dead-reckoning. Detects
maze walls (red roofs) AND robot pose (ArUco marker on top of robot), then
polls firmware /state telemetry and diffs both views in one viewer.

Workflow:
  0. Print + tape marker:  python3 detect_maze.py --print-marker
        Writes marker.png (ArUco 4x4_50 ID 0). Print actual-size, glue flat
        on top of robot. Marker's "top edge" must point ROBOT FORWARD.
  1. Calibrate once:       python3 detect_maze.py --calibrate
        Click 4 corners IN ORDER: SW, SE, NE, NW.
  2. Run live:             python3 detect_maze.py --firmware http://mm26.local/state
        Open http://localhost:5005. Keys: q quit, r tune HSV, c recalibrate,
        s snapshot, p print marker, h help.

Output JSON matches firmware PinConfig.h bits: N=1, E=2, S=4, W=8.
Row index increases NORTHWARD, col index increases EASTWARD.
AbsDir: 0=N, 1=E, 2=S, 3=W (matches firmware enum).
"""

import argparse
import json
import logging
import math
import os
import sys
import threading
import time

import cv2
import numpy as np
from flask import Flask, jsonify, request, send_from_directory

try:
    import requests
except ImportError:
    requests = None

ROWS = 6
COLS = 3
CELL_MM = 180.0                # standard micromouse cell pitch
CELL_PX = 100                  # warped pixels per cell side
W_PX = COLS * CELL_PX          # 300
H_PX = ROWS * CELL_PX          # 600
MM_PER_PX = CELL_MM / CELL_PX  # 1.8 mm/px

WALL_N, WALL_E, WALL_S, WALL_W = 1, 2, 4, 8
DIR_NAMES = ["N", "E", "S", "W"]

DEFAULT_HSV = {
    "lo1": [0, 110, 70],   "hi1": [10, 255, 255],
    "lo2": [170, 110, 70], "hi2": [180, 255, 255],
}

EDGE_THICKNESS_FRAC = 0.10
EDGE_PAD_FRAC = 0.18
WALL_RED_THRESH = 0.22

ARUCO_DICT_NAME = "DICT_4X4_50"
ROBOT_MARKER_ID = 0

# Green-blob fallback: detect a green sticker/tape on top of robot when ArUco
# isn't visible. Heading 180° ambiguous from blob alone — resolved using
# velocity history once the robot moves.
DEFAULT_GREEN_HSV = {"lo": [40, 80, 60], "hi": [85, 255, 255]}
GREEN_MIN_AREA_PX = 80

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CORNERS_FILE = os.path.join(SCRIPT_DIR, "corners.json")
HSV_FILE = os.path.join(SCRIPT_DIR, "hsv.json")
MAP_FILE = os.path.join(SCRIPT_DIR, "map.json")
MARKER_FILE = os.path.join(SCRIPT_DIR, "marker.png")
BG_REF_FILE = os.path.join(SCRIPT_DIR, "bg_ref.png")
BGSUB_THRESHOLD = 35
BGSUB_MIN_AREA_PX = 200

_state_lock = threading.Lock()
_latest_map = {"timestamp": 0, "rows": ROWS, "cols": COLS,
               "camera": {}, "firmware": {}, "diff": {}}
_latest_sim = None
_bg_capture_request = {"pending": False}

# Firmware poller thread state
_fw_lock = threading.Lock()
_fw_state = {"online": False, "url": None, "last_fetch": 0, "data": None, "error": None}


def load_json(path, default):
    if os.path.exists(path):
        with open(path) as f:
            return json.load(f)
    return default


def save_json(path, data):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(data, f, indent=2)
    os.replace(tmp, path)


# --- ArUco helpers ---------------------------------------------------------

def get_aruco():
    """Return (detector_callable, dictionary). Handles both old/new OpenCV APIs."""
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, ARUCO_DICT_NAME))
    if hasattr(cv2.aruco, "ArucoDetector"):
        params = cv2.aruco.DetectorParameters()
        det = cv2.aruco.ArucoDetector(dictionary, params)
        def detect(img):
            return det.detectMarkers(img)
        return detect, dictionary
    else:
        params = cv2.aruco.DetectorParameters_create()
        def detect(img):
            return cv2.aruco.detectMarkers(img, dictionary, parameters=params)
        return detect, dictionary


def write_marker_png(side_mm=70, dpi=300):
    """Write marker.png at print-ready size for ID=ROBOT_MARKER_ID."""
    _, dictionary = get_aruco()
    px = int(side_mm / 25.4 * dpi)
    img = cv2.aruco.generateImageMarker(dictionary, ROBOT_MARKER_ID, px)
    border = px // 8
    canvas = np.full((px + 2 * border, px + 2 * border), 255, dtype=np.uint8)
    canvas[border:border + px, border:border + px] = img
    cv2.imwrite(MARKER_FILE, canvas)
    print(f"Wrote {MARKER_FILE}: {side_mm}mm side, ID={ROBOT_MARKER_ID} ({ARUCO_DICT_NAME})")
    print(f"Print at {dpi} DPI, actual size. Glue flat on top of robot, top edge = forward.")


def _pose_from_pixel(cx, cy, dx, dy):
    bearing_rad = math.atan2(dx, -dy)
    bearing_deg = (math.degrees(bearing_rad) + 360.0) % 360.0
    heading_dir = int(round(bearing_deg / 90.0)) % 4
    x_mm = cx * MM_PER_PX
    y_mm = (H_PX - cy) * MM_PER_PX
    col = max(0, min(COLS - 1, int(cx // CELL_PX)))
    row = max(0, min(ROWS - 1, ROWS - 1 - int(cy // CELL_PX)))
    return {
        "x_mm": round(x_mm, 1),
        "y_mm": round(y_mm, 1),
        "row": row, "col": col,
        "heading_deg": round(bearing_deg, 1),
        "heading_dir": heading_dir,
        "heading_name": DIR_NAMES[heading_dir],
        "_px_cx": cx, "_px_cy": cy,
        "_px_fwd": [dx, dy],
    }


def _detect_aruco(warped_bgr, detector):
    gray = cv2.cvtColor(warped_bgr, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector(gray)
    if ids is None:
        return None
    for i, mid in enumerate(ids.flatten()):
        if int(mid) != ROBOT_MARKER_ID:
            continue
        pts = corners[i].reshape(4, 2)
        cx = float(pts[:, 0].mean()); cy = float(pts[:, 1].mean())
        top_mid = (pts[0] + pts[1]) / 2.0
        fwd = top_mid - np.array([cx, cy])
        pose = _pose_from_pixel(cx, cy, float(fwd[0]), float(fwd[1]))
        pose["detected"] = True
        pose["source"] = "aruco"
        return pose
    return None


def _detect_green(warped_bgr, hsv, prev_pose):
    """Green-blob fallback. Heading derived from minAreaRect; 180° ambiguity
    resolved via prev_pose direction continuity."""
    img_hsv = cv2.cvtColor(warped_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(img_hsv, np.array(hsv["lo"]), np.array(hsv["hi"]))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    cnt = max(contours, key=cv2.contourArea)
    if cv2.contourArea(cnt) < GREEN_MIN_AREA_PX:
        return None
    (cx_f, cy_f), (w, h), ang = cv2.minAreaRect(cnt)
    cx, cy = float(cx_f), float(cy_f)
    # minAreaRect ang ∈ [-90, 0]; convert to a unit vector along long axis.
    if w >= h:
        theta = math.radians(ang)
    else:
        theta = math.radians(ang + 90.0)
    dx, dy = math.cos(theta), math.sin(theta)
    # Resolve 180° ambiguity using prev_pose's forward vector (continuity).
    if prev_pose and prev_pose.get("detected"):
        pdx, pdy = prev_pose["_px_fwd"]
        n = math.hypot(pdx, pdy) or 1.0
        if dx * pdx / n + dy * pdy / n < 0:
            dx, dy = -dx, -dy
    pose = _pose_from_pixel(cx, cy, dx, dy)
    pose["detected"] = True
    pose["source"] = "green"
    pose["heading_uncertain"] = (prev_pose is None or not prev_pose.get("detected"))
    return pose


def _detect_bgsub(warped_bgr, bg_ref, prev_pose):
    """Background subtraction. Needs bg_ref captured with empty maze. Heading
    from frame-to-frame motion if available; otherwise uncertain."""
    if bg_ref is None or bg_ref.shape != warped_bgr.shape:
        return None
    diff = cv2.absdiff(warped_bgr, bg_ref)
    gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, BGSUB_THRESHOLD, 255, cv2.THRESH_BINARY)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((11, 11), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    cnt = max(contours, key=cv2.contourArea)
    if cv2.contourArea(cnt) < BGSUB_MIN_AREA_PX:
        return None
    M = cv2.moments(cnt)
    if M["m00"] == 0:
        return None
    cx = M["m10"] / M["m00"]
    cy = M["m01"] / M["m00"]
    if prev_pose and prev_pose.get("detected"):
        pdx = cx - prev_pose["_px_cx"]
        pdy = cy - prev_pose["_px_cy"]
        if math.hypot(pdx, pdy) > 3.0:
            dx, dy = pdx, pdy
        else:
            dx, dy = prev_pose["_px_fwd"]
    else:
        dx, dy = 0.0, -1.0
    pose = _pose_from_pixel(cx, cy, dx, dy)
    pose["detected"] = True
    pose["source"] = "bgsub"
    pose["heading_uncertain"] = True
    return pose


def detect_robot(warped_bgr, detector, hsv_green, bg_ref, prev_pose):
    """Try ArUco → green-blob → bgsub. Returns {detected: ...}."""
    p = _detect_aruco(warped_bgr, detector)
    if p:
        return p
    p = _detect_green(warped_bgr, hsv_green, prev_pose)
    if p:
        return p
    p = _detect_bgsub(warped_bgr, bg_ref, prev_pose)
    if p:
        return p
    return {"detected": False, "source": "none"}


# --- Firmware poller -------------------------------------------------------

def firmware_poller(url, interval_s):
    if requests is None:
        print("requests module missing; firmware poll disabled.")
        return
    while True:
        try:
            r = requests.get(url, timeout=0.5)
            r.raise_for_status()
            data = r.json()
            with _fw_lock:
                _fw_state["online"] = True
                _fw_state["url"] = url
                _fw_state["last_fetch"] = time.time()
                _fw_state["data"] = data
                _fw_state["error"] = None
        except Exception as e:
            with _fw_lock:
                _fw_state["online"] = False
                _fw_state["url"] = url
                _fw_state["error"] = str(e)
        time.sleep(interval_s)


def start_firmware_poller(url, interval_s=0.1):
    t = threading.Thread(target=firmware_poller, args=(url, interval_s), daemon=True)
    t.start()
    return t


# --- Flask server ----------------------------------------------------------

def build_app(firmware_base):
    """firmware_base: e.g. 'http://mm26.local' or None to disable proxy."""
    app = Flask(__name__, static_folder=SCRIPT_DIR, static_url_path="")
    logging.getLogger("werkzeug").setLevel(logging.WARNING)

    @app.route("/")
    def index():
        return send_from_directory(SCRIPT_DIR, "viewer.html")

    @app.route("/map.json")
    def map_json():
        with _state_lock:
            return jsonify(_latest_map)

    @app.route("/sim.json", methods=["GET", "POST"])
    def sim_json():
        global _latest_sim
        if request.method == "POST":
            _latest_sim = request.get_json(force=True, silent=True)
            return ("", 204)
        with _state_lock:
            return jsonify(_latest_sim or {})

    @app.route("/fw/cmd/<path:sub>", methods=["GET", "POST"])
    def fw_cmd_proxy(sub):
        """Proxy /fw/cmd/<x> → {firmware_base}/cmd/<x>. Avoids CORS in viewer."""
        if not firmware_base or requests is None:
            return jsonify({"ok": False, "err": "no firmware base set"}), 503
        url = firmware_base.rstrip("/") + "/cmd/" + sub
        try:
            r = requests.get(url, params=request.args.to_dict(), timeout=1.5)
            return (r.text, r.status_code,
                    {"Content-Type": r.headers.get("Content-Type", "application/json")})
        except Exception as e:
            return jsonify({"ok": False, "err": str(e)}), 502

    @app.route("/capture_bg", methods=["POST"])
    def capture_bg_route():
        with _state_lock:
            ok = _latest_map.get("_last_warped_b64") is None
        # Inform the detect loop to capture on next frame via a flag.
        _bg_capture_request["pending"] = True
        return jsonify({"ok": True, "msg": "will capture on next frame"})

    return app


def start_server(host, port, firmware_base):
    app = build_app(firmware_base)
    t = threading.Thread(
        target=lambda: app.run(host=host, port=port, debug=False,
                               use_reloader=False, threaded=True),
        daemon=True,
    )
    t.start()
    return t


# --- Camera + warp ---------------------------------------------------------

def open_camera(idx):
    cap = cv2.VideoCapture(idx)
    if not cap.isOpened():
        sys.exit(f"Cannot open camera index {idx}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def calibrate(cap):
    print("Click 4 corners IN ORDER: SW, SE, NE, NW. Backspace=undo, Enter=save, Esc=cancel.")
    pts = []

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(pts) < 4:
            pts.append([x, y])

    win = "calibrate (SW SE NE NW)"
    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)
    labels = ["SW", "SE", "NE", "NW"]
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        disp = frame.copy()
        for i, p in enumerate(pts):
            cv2.circle(disp, tuple(p), 8, (0, 255, 0), -1)
            cv2.putText(disp, labels[i], (p[0] + 10, p[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        if len(pts) < 4:
            cv2.putText(disp, f"Click {labels[len(pts)]}", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
        else:
            cv2.putText(disp, "Enter=save  Backspace=undo  Esc=cancel",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imshow(win, disp)
        k = cv2.waitKey(1) & 0xFF
        if k == 8 and pts:
            pts.pop()
        elif k == 13 and len(pts) == 4:
            save_json(CORNERS_FILE, {"sw": pts[0], "se": pts[1],
                                     "ne": pts[2], "nw": pts[3]})
            print(f"Saved corners → {CORNERS_FILE}")
            break
        elif k == 27:
            print("Cancelled")
            break
    cv2.destroyWindow(win)


def warp_matrix(corners):
    src = np.float32([corners["sw"], corners["se"], corners["ne"], corners["nw"]])
    dst = np.float32([[0, H_PX], [W_PX, H_PX], [W_PX, 0], [0, 0]])
    return cv2.getPerspectiveTransform(src, dst)


# --- Wall detection --------------------------------------------------------

def red_mask(bgr, hsv_cfg):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m1 = cv2.inRange(hsv, np.array(hsv_cfg["lo1"]), np.array(hsv_cfg["hi1"]))
    m2 = cv2.inRange(hsv, np.array(hsv_cfg["lo2"]), np.array(hsv_cfg["hi2"]))
    mask = cv2.bitwise_or(m1, m2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    return mask


def cell_xy(row, col):
    img_row = ROWS - 1 - row
    return col * CELL_PX, img_row * CELL_PX


def detect_walls(mask):
    walls = [[0] * COLS for _ in range(ROWS)]
    t = max(2, int(EDGE_THICKNESS_FRAC * CELL_PX))
    pad = int(EDGE_PAD_FRAC * CELL_PX)

    def strip_density(y0, y1, x0, x1):
        y0, y1 = max(0, y0), min(H_PX, y1)
        x0, x1 = max(0, x0), min(W_PX, x1)
        if y1 <= y0 or x1 <= x0:
            return 0.0
        return float(mask[y0:y1, x0:x1].mean()) / 255.0

    for r in range(ROWS):
        for c in range(COLS):
            x, y = cell_xy(r, c)
            d_n = strip_density(y - t, y + t, x + pad, x + CELL_PX - pad)
            d_s = strip_density(y + CELL_PX - t, y + CELL_PX + t,
                                x + pad, x + CELL_PX - pad)
            d_w = strip_density(y + pad, y + CELL_PX - pad, x - t, x + t)
            d_e = strip_density(y + pad, y + CELL_PX - pad,
                                x + CELL_PX - t, x + CELL_PX + t)
            if d_n > WALL_RED_THRESH: walls[r][c] |= WALL_N
            if d_s > WALL_RED_THRESH: walls[r][c] |= WALL_S
            if d_w > WALL_RED_THRESH: walls[r][c] |= WALL_W
            if d_e > WALL_RED_THRESH: walls[r][c] |= WALL_E

    for r in range(ROWS):
        for c in range(COLS):
            m = walls[r][c]
            if (m & WALL_N) and r + 1 < ROWS: walls[r + 1][c] |= WALL_S
            if (m & WALL_S) and r - 1 >= 0:   walls[r - 1][c] |= WALL_N
            if (m & WALL_E) and c + 1 < COLS: walls[r][c + 1] |= WALL_W
            if (m & WALL_W) and c - 1 >= 0:   walls[r][c - 1] |= WALL_E
    return walls


def ascii_grid(walls, mark_row=None, mark_col=None, mark_dir=None):
    arrows = ["^", ">", "v", "<"]
    lines = ["+" + "---+" * COLS]
    for r in range(ROWS - 1, -1, -1):
        mid = "|"
        for c in range(COLS):
            cell = "   "
            if mark_row == r and mark_col == c:
                a = arrows[mark_dir] if mark_dir is not None else "*"
                cell = f" {a} "
            mid += cell + ("|" if walls[r][c] & WALL_E else " ")
        if not (walls[r][0] & WALL_W):
            mid = " " + mid[1:]
        lines.append(mid)
        bot = "+"
        for c in range(COLS):
            bot += ("---" if walls[r][c] & WALL_S else "   ") + "+"
        lines.append(bot)
    return "\n".join(lines)


# --- Diff ------------------------------------------------------------------

def popcount(n):
    c = 0
    while n:
        c += n & 1
        n >>= 1
    return c


def compute_diff(cam_walls, cam_robot, fw_data):
    """Return dict of diffs between camera ground truth and firmware claim."""
    diff = {
        "wall_mismatches": None, "wall_diff": None,
        "cell_match": None, "pose_error_mm": None, "pose_error_deg": None,
        "fw_pose_px": None,
    }
    if not fw_data:
        return diff

    fw_walls = fw_data.get("walls")
    if isinstance(fw_walls, list) and len(fw_walls) >= ROWS and len(fw_walls[0]) >= COLS:
        wd = [[0] * COLS for _ in range(ROWS)]
        n = 0
        for r in range(ROWS):
            for c in range(COLS):
                x = (cam_walls[r][c] ^ int(fw_walls[r][c])) & 0xF
                wd[r][c] = x
                n += popcount(x)
        diff["wall_diff"] = wd
        diff["wall_mismatches"] = n

    fw_row = fw_data.get("row")
    fw_col = fw_data.get("col")
    fw_heading = fw_data.get("heading")
    if fw_row is not None and fw_col is not None:
        fw_cx_mm = (fw_col + 0.5) * CELL_MM
        fw_cy_mm = (fw_row + 0.5) * CELL_MM     # from south
        fw_cx_px = fw_cx_mm / MM_PER_PX
        fw_cy_px = H_PX - fw_cy_mm / MM_PER_PX
        diff["fw_pose_px"] = [fw_cx_px, fw_cy_px]
        if cam_robot and cam_robot.get("detected"):
            dx = cam_robot["x_mm"] - fw_cx_mm
            dy = cam_robot["y_mm"] - fw_cy_mm
            diff["pose_error_mm"] = round(math.hypot(dx, dy), 1)
            diff["cell_match"] = (cam_robot["row"] == fw_row and
                                  cam_robot["col"] == fw_col)
            if fw_heading is not None:
                fw_deg = (int(fw_heading) % 4) * 90.0
                err = (cam_robot["heading_deg"] - fw_deg + 540.0) % 360.0 - 180.0
                diff["pose_error_deg"] = round(err, 1)
    return diff


# --- Overlay ---------------------------------------------------------------

def overlay_debug(warped, mask, walls, cam_robot, diff):
    out = warped.copy()
    red_tint = np.zeros_like(out); red_tint[..., 2] = mask
    out = cv2.addWeighted(out, 0.7, red_tint, 0.3, 0)
    wd = (diff or {}).get("wall_diff")
    for r in range(ROWS):
        for c in range(COLS):
            x, y = cell_xy(r, c)
            cv2.rectangle(out, (x, y), (x + CELL_PX, y + CELL_PX), (60, 60, 60), 1)
            m = walls[r][c]
            mism = (wd[r][c] if wd else 0)
            def col(side):
                if mism & side: return (0, 0, 255)
                return (0, 255, 0)
            if m & WALL_N: cv2.line(out, (x, y), (x + CELL_PX, y), col(WALL_N), 3)
            if m & WALL_S: cv2.line(out, (x, y + CELL_PX), (x + CELL_PX, y + CELL_PX), col(WALL_S), 3)
            if m & WALL_W: cv2.line(out, (x, y), (x, y + CELL_PX), col(WALL_W), 3)
            if m & WALL_E: cv2.line(out, (x + CELL_PX, y), (x + CELL_PX, y + CELL_PX), col(WALL_E), 3)
            cv2.putText(out, f"{r},{c}", (x + 4, y + 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 0), 1)
    # firmware-claimed pose (blue)
    fpx = (diff or {}).get("fw_pose_px")
    if fpx:
        cv2.circle(out, (int(fpx[0]), int(fpx[1])), 14, (255, 120, 0), 2)
        cv2.putText(out, "FW", (int(fpx[0]) - 12, int(fpx[1]) - 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 120, 0), 2)
    # camera-detected pose (green arrow)
    if cam_robot and cam_robot.get("detected"):
        cx, cy = int(cam_robot["_px_cx"]), int(cam_robot["_px_cy"])
        fdx, fdy = cam_robot["_px_fwd"]
        L = 40.0
        n = math.hypot(fdx, fdy) or 1.0
        ex, ey = int(cx + fdx / n * L), int(cy + fdy / n * L)
        cv2.circle(out, (cx, cy), 10, (0, 255, 100), -1)
        cv2.arrowedLine(out, (cx, cy), (ex, ey), (0, 255, 100), 3, tipLength=0.3)
        cv2.putText(out, "CAM", (cx + 12, cy + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 100), 2)
    return out


def hsv_tuner(cap, hsv_cfg, corners):
    win = "HSV tune — s=save  q=cancel"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    def nop(_): pass
    cv2.createTrackbar("H1 lo", win, hsv_cfg["lo1"][0], 180, nop)
    cv2.createTrackbar("H1 hi", win, hsv_cfg["hi1"][0], 180, nop)
    cv2.createTrackbar("H2 lo", win, hsv_cfg["lo2"][0], 180, nop)
    cv2.createTrackbar("H2 hi", win, hsv_cfg["hi2"][0], 180, nop)
    cv2.createTrackbar("S min", win, hsv_cfg["lo1"][1], 255, nop)
    cv2.createTrackbar("V min", win, hsv_cfg["lo1"][2], 255, nop)
    M = warp_matrix(corners) if corners else None
    while True:
        ok, frame = cap.read()
        if not ok: continue
        view = cv2.warpPerspective(frame, M, (W_PX, H_PX)) if M is not None else frame
        cfg = {
            "lo1": [cv2.getTrackbarPos("H1 lo", win),
                    cv2.getTrackbarPos("S min", win),
                    cv2.getTrackbarPos("V min", win)],
            "hi1": [cv2.getTrackbarPos("H1 hi", win), 255, 255],
            "lo2": [cv2.getTrackbarPos("H2 lo", win),
                    cv2.getTrackbarPos("S min", win),
                    cv2.getTrackbarPos("V min", win)],
            "hi2": [cv2.getTrackbarPos("H2 hi", win), 255, 255],
        }
        mask = red_mask(view, cfg)
        combo = np.hstack([view, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)])
        cv2.imshow(win, combo)
        k = cv2.waitKey(30) & 0xFF
        if k == ord('s'):
            save_json(HSV_FILE, cfg)
            print(f"Saved HSV → {HSV_FILE}")
            cv2.destroyWindow(win)
            return cfg
        if k == ord('q') or k == 27:
            cv2.destroyWindow(win)
            return hsv_cfg


# --- Main ------------------------------------------------------------------

def strip_internal(cam_robot):
    """Drop keys starting with _ before sending to JSON."""
    if not cam_robot:
        return cam_robot
    return {k: v for k, v in cam_robot.items() if not k.startswith("_")}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--calibrate", action="store_true")
    ap.add_argument("--tune-hsv", action="store_true")
    ap.add_argument("--print-marker", action="store_true",
                    help="generate marker.png and exit")
    ap.add_argument("--marker-mm", type=float, default=70.0,
                    help="marker side length in mm (default 70)")
    ap.add_argument("--no-window", action="store_true")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5005)
    ap.add_argument("--no-server", action="store_true")
    ap.add_argument("--no-file", action="store_true")
    ap.add_argument("--firmware", default=None,
                    help="firmware /state URL, e.g. http://mm26.local/state")
    ap.add_argument("--firmware-hz", type=float, default=10.0)
    args = ap.parse_args()

    if args.print_marker:
        write_marker_png(side_mm=args.marker_mm)
        return

    cap = open_camera(args.camera)
    hsv_cfg = load_json(HSV_FILE, DEFAULT_HSV)
    detector, _ = get_aruco()

    if args.calibrate:
        calibrate(cap)

    corners = load_json(CORNERS_FILE, None)
    if corners is None:
        print("No corners.json — calibrating now.")
        calibrate(cap)
        corners = load_json(CORNERS_FILE, None)
        if corners is None:
            sys.exit("Calibration aborted.")

    if args.tune_hsv:
        hsv_cfg = hsv_tuner(cap, hsv_cfg, corners)

    M = warp_matrix(corners)

    if args.firmware:
        start_firmware_poller(args.firmware, 1.0 / max(0.5, args.firmware_hz))
        print(f"Polling firmware {args.firmware} @ {args.firmware_hz} Hz")

    # Derive firmware base URL (drop /state) for the cmd proxy.
    firmware_base = None
    if args.firmware:
        firmware_base = args.firmware.split("/state")[0].rstrip("/")

    if not args.no_server:
        start_server(args.host, args.port, firmware_base)
        proxy = f"  /fw/cmd/* → {firmware_base}/cmd/*" if firmware_base else ""
        print(f"Server: http://{args.host}:{args.port}/   "
              f"endpoints: /map.json /sim.json{proxy}")

    bg_ref = cv2.imread(BG_REF_FILE) if os.path.exists(BG_REF_FILE) else None
    if bg_ref is not None:
        print(f"Loaded bg ref: {BG_REF_FILE}")
    print("Detect loop. Keys: q quit, r tune HSV, c recalibrate, s snapshot, "
          "p print marker, b capture bg (clear maze first!).")
    last_print = 0.0
    prev_pose = None

    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        warped = cv2.warpPerspective(frame, M, (W_PX, H_PX))
        mask = red_mask(warped, hsv_cfg)
        walls = detect_walls(mask)
        cam_robot = detect_robot(warped, detector, DEFAULT_GREEN_HSV, bg_ref, prev_pose)
        if cam_robot.get("detected"):
            prev_pose = cam_robot

        with _fw_lock:
            fw_snapshot = dict(_fw_state)
        fw_data = fw_snapshot.get("data")

        diff = compute_diff(walls, cam_robot, fw_data)

        out = {
            "timestamp": time.time(),
            "rows": ROWS, "cols": COLS,
            "wall_bits": {"N": WALL_N, "E": WALL_E, "S": WALL_S, "W": WALL_W},
            "cell_mm": CELL_MM,
            "row_axis": "row 0 = SOUTH, row ROWS-1 = NORTH",
            "col_axis": "col 0 = WEST,  col COLS-1 = EAST",
            "abs_dir": {"0": "N", "1": "E", "2": "S", "3": "W"},
            "camera": {
                "walls": walls,
                "robot": strip_internal(cam_robot),
            },
            "firmware": {
                "online": fw_snapshot["online"],
                "url": fw_snapshot["url"],
                "last_fetch_age_s": (time.time() - fw_snapshot["last_fetch"])
                                    if fw_snapshot["last_fetch"] else None,
                "error": fw_snapshot["error"],
                "row": fw_data.get("row") if fw_data else None,
                "col": fw_data.get("col") if fw_data else None,
                "heading": fw_data.get("heading") if fw_data else None,
                "yaw_deg": fw_data.get("yaw") if fw_data else None,
                "state": fw_data.get("state") if fw_data else None,
                "phase": fw_data.get("phase") if fw_data else None,
                "vbat": fw_data.get("vbat") if fw_data else None,
                "vL": fw_data.get("vL") if fw_data else None,
                "vR": fw_data.get("vR") if fw_data else None,
                "frontMM": fw_data.get("frontMM") if fw_data else None,
                "walls": fw_data.get("walls") if fw_data else None,
                "flood": fw_data.get("flood") if fw_data else None,
                "visited": fw_data.get("visited") if fw_data else None,
                "crash": fw_data.get("crash") if fw_data else None,
                "tune":  fw_data.get("tune")  if fw_data else None,
                "ir":    fw_data.get("ir")    if fw_data else None,
                "ir_phys": fw_data.get("ir_phys") if fw_data else None,
                "ir_layout": fw_data.get("ir_layout") if fw_data else None,
                "cal":   fw_data.get("cal")   if fw_data else None,
                "wall_front": fw_data.get("wall_front") if fw_data else None,
                "wall_left":  fw_data.get("wall_left")  if fw_data else None,
                "wall_right": fw_data.get("wall_right") if fw_data else None,
            },
            "diff": diff,
        }
        with _state_lock:
            _latest_map.clear()
            _latest_map.update(out)
        if not args.no_file:
            save_json(MAP_FILE, out)

        now = time.time()
        if now - last_print > 1.0:
            r = cam_robot.get("row") if cam_robot.get("detected") else None
            c = cam_robot.get("col") if cam_robot.get("detected") else None
            d = cam_robot.get("heading_dir") if cam_robot.get("detected") else None
            print("\n" + ascii_grid(walls, r, c, d))
            if fw_data:
                pe = diff.get("pose_error_mm"); de = diff.get("pose_error_deg")
                wm = diff.get("wall_mismatches"); cm = diff.get("cell_match")
                print(f"  FW r{fw_data.get('row')} c{fw_data.get('col')} "
                      f"h{fw_data.get('heading')} | CAM "
                      f"r{r} c{c} h{d} | "
                      f"cell_match={cm} dpos={pe}mm dhead={de}° wall_mis={wm}")
            last_print = now

        if not args.no_window:
            cv2.imshow("warped + robot", overlay_debug(warped, mask, walls, cam_robot, diff))
            k = cv2.waitKey(1) & 0xFF
            if k == ord('q'):
                break
            elif k == ord('r'):
                hsv_cfg = hsv_tuner(cap, hsv_cfg, corners)
            elif k == ord('c'):
                calibrate(cap)
                corners = load_json(CORNERS_FILE, corners)
                M = warp_matrix(corners)
            elif k == ord('s'):
                ts = int(time.time())
                cv2.imwrite(os.path.join(SCRIPT_DIR, f"snap_{ts}_warped.png"), warped)
                cv2.imwrite(os.path.join(SCRIPT_DIR, f"snap_{ts}_mask.png"), mask)
                print(f"snap_{ts}_*.png saved")
            elif k == ord('p'):
                write_marker_png(side_mm=args.marker_mm)
            elif k == ord('b'):
                cv2.imwrite(BG_REF_FILE, warped)
                bg_ref = warped.copy()
                print(f"Background reference captured → {BG_REF_FILE}")

        # Web-driven bg capture (POST /capture_bg).
        if _bg_capture_request["pending"]:
            _bg_capture_request["pending"] = False
            cv2.imwrite(BG_REF_FILE, warped)
            bg_ref = warped.copy()
            print(f"Background reference captured (web) → {BG_REF_FILE}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

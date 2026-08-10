# vision/ (optional)

OpenCV + Flask helpers for maze camera experiments. **Not required** for
firmware builds.

## Setup

```bash
python3 -m venv tools/.venv
source tools/.venv/bin/activate
pip install -r tools/vision/requirements.txt
```

## Notes

- Default Flask bind is `127.0.0.1`. Do not expose `--host 0.0.0.0` on an
  untrusted network — the `/fw/cmd` proxy can forward commands to the robot.
- `map.json`, `corners.json`, and similar runtime artifacts are gitignored.
  Commit only example/calibration templates if you add them intentionally.

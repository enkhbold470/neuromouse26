# PlatformIO post-upload hook: play a short system chime after a successful flash.
# Upload-only — does not run during `pio run` compile CI.
Import("env")
import shutil
import subprocess
import sys

def after_upload(source, target, env):
    try:
        if sys.platform == "darwin":
            subprocess.Popen(["afplay", "/System/Library/Sounds/Glass.aiff"])
        elif sys.platform == "win32":
            # Optional: pip install playsound. Failures are non-fatal.
            try:
                from playsound import playsound  # type: ignore
                # Prefer a local WAV if the user drops one next to this script.
                import os
                wav = os.path.join(os.path.dirname(__file__), "upload-chime.wav")
                if os.path.isfile(wav):
                    playsound(wav)
            except Exception as e:
                print(f"[notify] Windows sound skipped: {e}")
        else:
            # Linux: try paplay / aplay, else terminal bell.
            for cmd in (("paplay", "/usr/share/sounds/freedesktop/stereo/complete.oga"),
                        ("aplay", "/usr/share/sounds/alsa/Front_Left.wav")):
                if shutil.which(cmd[0]):
                    subprocess.Popen([cmd[0], cmd[1]],
                                     stdout=subprocess.DEVNULL,
                                     stderr=subprocess.DEVNULL)
                    return
            print("\a", end="", flush=True)
    except Exception as e:
        print(f"[notify] sound failed: {e}")

env.AddPostAction("upload", after_upload)

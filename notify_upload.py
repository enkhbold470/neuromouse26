Import("env")
import subprocess
import sys
import os

MP3 = os.path.join(env["PROJECT_DIR"], "universfield-new-notification-036-485897.mp3")

def after_upload(source, target, env):
    try:
        if sys.platform == "win32":
            from playsound import playsound
            playsound(MP3)
        elif sys.platform == "darwin":
            subprocess.Popen(["afplay", "/System/Library/Sounds/Glass.aiff"])
        else:
            print("\a", end="", flush=True)
    except Exception as e:
        print(f"[notify] sound failed: {e}")

env.AddPostAction("upload", after_upload)

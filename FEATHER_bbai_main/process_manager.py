import os
import subprocess
import sys
import time

# Ensure the working directory is the script's directory
os.chdir(os.path.dirname(os.path.abspath(__file__)))

# Paths to your scripts
IMU_SHARED = [sys.executable, "IMU_forwarder.py"]
FES_CONTROL = [sys.executable, "FEScontrol_1IMU.py"]
FEATHER_CONTROL = [sys.executable, "FEATHER_control.py"]

# Start all processes
procs = []
try:
    procs.append(subprocess.Popen(IMU_SHARED))
    time.sleep(1)  # Give IMU_shared time to start
    procs.append(subprocess.Popen(FEATHER_CONTROL))
    procs.append(subprocess.Popen(FES_CONTROL))

    print("All programs started. Press Ctrl+C to stop all.")

    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\nStopping all programs...")
    for p in procs:
        p.terminate()
    for p in procs:
        p.wait()
    print("All programs stopped.")
    sys.exit(0)
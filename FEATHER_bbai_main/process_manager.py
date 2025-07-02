import subprocess
import sys
import time

# Paths to your scripts
IMU_SHARED = "python3 IMU_shared.py"
FES_CONTROL = "python3 FEScontrol_1IMU.py"
FEATHER_CONTROL = "python3 FEATHER_control.py"

# Start all processes
procs = []
try:
    procs.append(subprocess.Popen(IMU_SHARED.split()))
    time.sleep(1)  # Give IMU_shared time to start
    procs.append(subprocess.Popen(FEATHER_CONTROL.split()))
    procs.append(subprocess.Popen(FES_CONTROL.split()))

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
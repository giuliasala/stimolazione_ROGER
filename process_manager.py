import os
import subprocess
import sys
import time

# Ensure the working directory is the script's directory
os.chdir(os.path.dirname(os.path.abspath(__file__)))

# Paths to your scripts
IMU1_SHARED = [sys.executable, "1IMU_forwarder.py"]
IMU2_SHARED = [sys.executable, "2IMU_forwarder.py"]
FES_CONTROL_1IMU = [sys.executable, "FEScontrol_1IMU.py"]
FES_CONTROL_2IMU = [sys.executable, "FEScontrol_2IMU.py"]
FEATHER_CONTROL = [sys.executable, "FEATHER_control.py"]

def start_processes(cmds):
    procs = []
    try:
        for i, cmd in enumerate(cmds):
            procs.append(subprocess.Popen(cmd))
            if i == 0:
                time.sleep(1)  # Give IMU forwarder time to start
        print("Starting all programs. Press Ctrl+C to stop all.")
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

n_imus = input("Are you using 1 or 2 IMUs?")

if n_imus == "1":
    start_processes([IMU1_SHARED, FEATHER_CONTROL, FES_CONTROL_1IMU])
elif n_imus == "2":
    start_processes([IMU2_SHARED, FEATHER_CONTROL, FES_CONTROL_2IMU])
else:
    print("Invalid input. Please enter 1 or 2.")

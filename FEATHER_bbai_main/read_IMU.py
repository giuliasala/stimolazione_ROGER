import numpy as np
import threading
import time

import utils

from tdu import Imu

# IMU parameters from NGIMU GUI
IMU_AXIS_UP = 'Y'
IMU_RECEIVE_PORTS = 8102
IMU_SEND_PORT = 9000
IMU_IP_ADDRESSES = "192.168.1.3" # in AP mode

# Thread Lock
lock = threading.Lock()

class systemState():
    UA_mat = np.matrix([[1,0,0],[0,1,0],[0,0,1]])
    sh_el = 0
    max_sh_el = 0

def compute_joint_angles(UA_mat):
    # Get the current shoulder elevation angle
    sh_el = np.arccos(UA_mat[2,1]) #element z of y axis
    return sh_el

class readImuLoop(threading.Thread):
    def __init__(self, name, system_state, imu, muscle, calibration_mode=False, duration=10):
        threading.Thread.__init__(self)
        self.name = name
        self.system_state = system_state
        self.imu = imu
        self.imu_fs = 200 # Need to read faster than IMU update frequency
        self.muscle = muscle
        self.calibration_mode = calibration_mode
        self.duration = duration # Time for calibration

    def run(self):
        print("Starting IMU reading thread...")
        
        dt = 1.0 / self.imu_fs
        self.imu.initialize(dt)
        self.imu.identify() # Strobe IMU leds to identify it
        IMU_mat = np.matrix([[1,0,0],[0,1,0],[0,0,1]])

        start_time = time.time()
        
        while True:

            if self.calibration_mode and time.time() - start_time > self.duration:
                break  # Stop after duration if in calibration mode

            next_time_instant = time.perf_counter() + dt
            # Get the IMUs rotation matrices
            try:
                IMU_m = self.imu.read_imu()
                IMU_mat = np.matrix([[IMU_m[0],IMU_m[1],IMU_m[2]],
                                     [IMU_m[3],IMU_m[4],IMU_m[5]],
                                     [IMU_m[6],IMU_m[7],IMU_m[8]]])
            except Exception as e:
                print("IMU read error:", e)
                pass
                
            with lock:
                
                self.system_state.UA_mat = IMU_mat
                self.system_state.sh_el = compute_joint_angles(IMU_mat)
                
                if self.calibration_mode:
                    self.system_state.max_sh_el = max(self.system_state.max_sh_el, self.system_state.sh_el)
              
            sh_el_deg = np.degrees(self.system_state.sh_el)
            print(f"Shoulder Elevation (deg): {sh_el_deg:.2f}")

            time.sleep(max(next_time_instant - time.perf_counter(), 0))

        if self.calibration_mode:
            max_sh_el_deg = np.degrees(self.system_state.max_sh_el)
            max_sh_el_deg = np.minimum(max_sh_el_deg, 130.0) # Cap at 130 to avoid singularity at 170 degrees (by Elena)
            
            if self.muscle == "a":
                filename = "anterior_calibration_data.json"
            elif self.muscle == "m":
                filename = "middle_calibration_data.json"
            utils.save_to_json(filename, max_sh_el_deg, "max_angle")
            
            print("Calibration complete. Max angle is:", max_sh_el_deg)
        
        print("IMU thread finished.")
            
def calibrate():        
    
    muscle = input("Do you want to stimulate anterior(a) or middle(m) deltoid? ").lower().strip()
    duration_str = input("Duration (number): ")
    duration = int(duration_str)

    readImuThread = readImuLoop("Read IMU", system_state, imu, muscle, True, duration)
    readImuThread.start()

if __name__ == "__main__":

    system_state = systemState()
    imu = Imu(IMU_RECEIVE_PORTS, IMU_IP_ADDRESSES, IMU_SEND_PORT, IMU_AXIS_UP)
    calibration_mode = input("Start calibration? (y/n) ").lower().strip()
    
    if calibration_mode == "y":
        calibrate()
    else:
        readImuThread = readImuLoop("Read IMU", system_state, imu, False)
        readImuThread.start()
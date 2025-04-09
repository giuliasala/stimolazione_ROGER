#!/usr/bin/python3

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
    initial_UA_mat = None
    sh_el = 0
    max_sh_el = 0

def compute_joint_angles(UA_mat):
    # Get the current shoulder elevation angle
    sh_el = np.arccos(UA_mat[2,1]) #element z of y axis
    return sh_el

class readImuLoop(threading.Thread):
    def __init__(self, name, system_state, imu, filename):
        threading.Thread.__init__(self)
        self.name = name
        self.system_state = system_state
        self.imu = imu
        self.imu_fs = 200 # Need to read faster than IMU update frequency
        self.dt = 1.0 / self.imu_fs
        self.filename = filename
        self.duration = 3 # of the movement

    def read_imu_matrix(self):
        # Get the IMUs rotation matrix
        try:
            IMU_m = self.imu.read_imu()
            return np.matrix([[IMU_m[0], IMU_m[1], IMU_m[2]],
                              [IMU_m[3], IMU_m[4], IMU_m[5]],
                              [IMU_m[6], IMU_m[7], IMU_m[8]]])
        except Exception as e:
            print("IMU read error:", e)
            return None
    
    def pre_calibrate(self):
        print(f"Starting pre-calibration. Please stay still for {self.duration} seconds...")
        matrices = []
        start_time = time.time()

        while time.time() - start_time < self.duration:
            next_time_instant = time.perf_counter() + self.dt
            
            IMU_mat = self.read_imu_matrix()
            if IMU_mat is not None:
                matrices.append(IMU_mat)
            
            time.sleep(max(next_time_instant - time.perf_counter(), 0))
        
        # Compute the average matrix for pre-calibration
        if matrices:
            avg_matrix = sum(matrices) / len(matrices)
            with lock:
                self.system_state.initial_UA_mat = avg_matrix
            print("\nPre-calibration complete. Initial matrix saved.")
        else:
            print("\nPre-calibration failed. No data recorded.")
            raise RuntimeError("Pre-calibration failed. Exiting thread.")
    
    def run(self):
        print("Starting IMU reading thread...")
        
        self.imu.initialize(self.dt)
        self.imu.identify() # Strobe IMU leds to identify it
        IMU_mat = np.matrix([[1,0,0],[0,1,0],[0,0,1]])
        
        # Pre-calibration phase
        try:
            self.pre_calibrate()
        except RuntimeError:
            return

        # Main loop
        start_time = time.time()
        
        while True:
            if time.time() - start_time > self.duration:
                break  # Stop after duration if in calibration mode

            next_time_instant = time.perf_counter() + self.dt
            
            IMU_mat = self.read_imu_matrix()
            if IMU_mat is not None:
                with lock:
                    if self.system_state.initial_UA_mat is not None:
                        rel_UA_mat = self.system_state.inital_UA_mat.I @ IMU_mat
                    else:
                        rel_UA_mat = IMU_mat # Fallback if pre-calibration failed
                        
                    self.system_state.UA_mat = rel_UA_mat
                    self.system_state.sh_el = compute_joint_angles(rel_UA_mat)
                    self.system_state.max_sh_el = max(self.system_state.max_sh_el, self.system_state.sh_el)
                
                sh_el_deg = np.degrees(self.system_state.sh_el)
                print(f"Shoulder Elevation (deg): {sh_el_deg:.2f}")

            time.sleep(max(next_time_instant - time.perf_counter(), 0))
        
        max_sh_el_deg = np.degrees(self.system_state.max_sh_el)
        max_sh_el_deg = np.minimum(max_sh_el_deg, 130.0) # Cap at 130 to avoid singularity at 170 degrees (by Elena)
        print(f"Maximum Shoulder Elevation (deg): {max_sh_el_deg:.2f}")
        
        utils.save_to_json(self.filename, max_sh_el_deg, "max_angle")
        
        print("Calibration complete.")

if __name__ == "__main__":

    system_state = systemState()
    imu = Imu(IMU_RECEIVE_PORTS, IMU_IP_ADDRESSES, IMU_SEND_PORT, IMU_AXIS_UP)
    
    user = input("Your name: ").lower().strip()
    muscle = input("Do you want to stimulate anterior(a) or middle(m) deltoid? ").lower().strip()
    if muscle == "a":
        filename = f"{user}_anterior_calibration_data.json"
    elif muscle == "m":
        filename = f"{user}_middle_calibration_data.json"
    else:
        print("Invalid input. Exiting.")
        exit()

    readImuThread = readImuLoop("Read IMU", system_state, imu, filename)    
    readImuThread.start()
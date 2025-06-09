#!/usr/bin/python3

import numpy as np
import threading
import time

import utils

from tdu import Imu

# IMU parameters from NGIMU GUI
IMU_AXIS_UP = 'Y'
IMU_RECEIVE_PORT = 8102
IMU_SEND_PORT = 9000
IMU_IP_ADDRESS = "192.168.1.2" # in AP mode

# Thread Lock
lock = threading.Lock()

def compute_joint_angles(UA_mat):
    # Get the current shoulder elevation angle
    sh_el = np.arccos(UA_mat[2,1]) #element z of y axis
    return sh_el

class readImuLoop(threading.Thread):
    def __init__(self, name, imu, filename):
        threading.Thread.__init__(self)
        self.name = name
        self.imu = imu
        self.imu_fs = 200 # Need to read faster than IMU update frequency
        self.dt = 1.0 / self.imu_fs
        self.filename = filename
        self.duration = 3 # of the movement

        self.initial_sh_el_array = []
        self.initial_sh_el = 0
        self.max_sh_el_array = []

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
        start_time = time.time()

        while time.time() - start_time < self.duration:
            next_time_instant = time.perf_counter() + self.dt
            
            IMU_mat = self.read_imu_matrix()
            sh_el = compute_joint_angles(IMU_mat)
            self.initial_sh_el_array.append(sh_el)
            
            time.sleep(max(next_time_instant - time.perf_counter(), 0))
        
        self.initial_sh_el = np.median(self.initial_sh_el_array)
        initial_sh_el_deg = np.degrees(self.initial_sh_el)
        print(f"Pre-calibration Shoulder Elevation (deg): {initial_sh_el_deg:.2f}")
        
    def run(self):
        print("Starting IMU reading thread...")
        
        self.imu.initialize(self.dt)
        self.imu.identify() # Strobe IMU leds to identify it
        IMU_mat = np.matrix([[1,0,0],[0,1,0],[0,0,1]])
        
        # Pre-calibration phase
        try:
            self.pre_calibrate()
        except RuntimeError:
            print("Pre-calibration failed.")
            return
        utils.save_to_json(self.filename, round(self.initial_sh_el, 3), "precalibration_angle (rad)")

        for rep in range(3):
            print(f"Rep {rep+1}")
            self.max_sh_el = 0
            start_time = time.time()
        
            while time.time() - start_time < self.duration:
                next_time_instant = time.perf_counter() + self.dt
                
                IMU_mat = self.read_imu_matrix()
                sh_el = compute_joint_angles(IMU_mat) - self.initial_sh_el
                sh_el_deg = np.degrees(sh_el)
                self.max_sh_el = max(self.max_sh_el, sh_el)
                
                print(f"Shoulder Elevation (deg): {sh_el_deg:.2f}")

                time.sleep(max(next_time_instant - time.perf_counter(), 0))
            
            max_sh_el_deg = np.degrees(self.max_sh_el)
            max_sh_el_deg = np.minimum(max_sh_el_deg, 130.00) # Cap at 130 to avoid singularity at 170 degrees (by Elena)
            print(f"Maximum Shoulder Elevation (deg) for rep {rep+1}: {max_sh_el_deg:.2f}")
            self.max_sh_el_array.append(max_sh_el_deg)
        
        final_max_angle = np.mean(self.max_sh_el_array)
        utils.save_to_json(self.filename, round(final_max_angle, 3), "max_angle (deg)")
            
        print("Calibration complete.")

if __name__ == "__main__":

    imu = Imu(IMU_RECEIVE_PORT, IMU_IP_ADDRESS, IMU_SEND_PORT, IMU_AXIS_UP)
    
    user = input("Your name: ").lower().strip()
    muscle = input("Do you want to stimulate anterior(a) or middle(m) deltoid? ").lower().strip()
    if muscle == "a":
        filename = f"{user}_anterior_calibration_data.json"
    elif muscle == "m":
        filename = f"{user}_middle_calibration_data.json"
    else:
        print("Invalid input. Exiting.")
        exit()

    readImuThread = readImuLoop("Read IMU", imu, filename)    
    readImuThread.start()
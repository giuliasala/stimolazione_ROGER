#!/usr/bin/python3

import numpy as np
import threading
import time

import utils

from tdu import Imu

# IMU parameters from NGIMU GUI
IMU_AXIS_UP = 'Y'
IMU_RECEIVE_PORTS = [8100, 8102]
IMU_SEND_PORT = 9000
IMU_IP_ADDRESSES = ["192.168.0.101","192.168.0.102"] # in client mode

# Thread Lock
lock = threading.Lock()

def compute_joint_angles(UA_mat):
    # Get the current shoulder elevation angle
    sh_el = np.arccos(UA_mat[2,1]) #element z of y axis
    return sh_el

class readImuLoop(threading.Thread):
    def __init__(self, name, imu, filename, contralateral, pre_calibration_only=False):
        threading.Thread.__init__(self)
        self.name = name
        self.imu = imu
        self.imu_fs = 200 # Need to read faster than IMU update frequency
        self.dt = 1.0 / self.imu_fs
        self.filename = filename
        self.duration = 4 # of the movement
        self.pre_duration = 3 # for pre-calibration
        self.contralateral = contralateral
        self.pre_calibration_only = pre_calibration_only

        self.initial_sh_el_array = []
        self.initial_sh_el = 0
        self.max_sh_el_array = []
        self.duration_array = []

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
        time.sleep(2) # temporary, to let me adjust when doing trials
        print(f"Starting pre-calibration. Please stay still for {self.pre_duration} seconds...")
        start_time = time.time()

        while time.time() - start_time < self.pre_duration:
            next_time_instant = time.perf_counter() + self.dt
            
            IMU_mat = self.read_imu_matrix()
            sh_el = compute_joint_angles(IMU_mat)
            self.initial_sh_el_array.append(sh_el)
            
            time.sleep(max(next_time_instant - time.perf_counter(), 0))
        
        self.initial_sh_el = np.mean(self.initial_sh_el_array)
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
        if self.contralateral:
            utils.save_to_json(self.filename, round(self.initial_sh_el, 3), "contralateral_precalibration_angle (rad)")
        else:
            utils.save_to_json(self.filename, round(self.initial_sh_el, 3),"precalibration_angle (rad)")
            if self.pre_calibration_only:
                return

            # 3 ripetizioni per estrarre l'angolo massimo raggiungibile e durata del movimento
            for rep in range(3):
                input(f"\nReady to start repetition {rep + 1}? Press Enter to continue...")
                self.max_sh_el = 0
                angle_trace = []  # List to store (timestamp, angle_deg)
                last_sh_el = None
                velocity = 0
                start_time = time.time()
            
                while time.time() - start_time < self.duration:
                    next_time_instant = time.perf_counter() + self.dt
                    
                    IMU_mat = self.read_imu_matrix()
                    sh_el = compute_joint_angles(IMU_mat) - self.initial_sh_el
                    sh_el_deg = np.degrees(sh_el)
                    self.max_sh_el = max(self.max_sh_el, sh_el)
                    #print(f"Shoulder Elevation (deg): {sh_el_deg:.2f}")

                    if last_sh_el is not None:
                        velocity = (sh_el_deg - last_sh_el) / self.dt
                        print(f"Velocity:{velocity:.2f} deg/s")
                    last_sh_el = sh_el_deg

                    angle_trace.append((time.time(), sh_el_deg, velocity))

                    time.sleep(max(next_time_instant - time.perf_counter(), 0))
                
                max_sh_el_deg = np.degrees(self.max_sh_el)
                max_sh_el_deg = np.minimum(max_sh_el_deg, 130.00) # Cap at 130 to avoid singularity at 170 degrees (by Elena)
                print(f"Maximum Shoulder Elevation (deg) for rep {rep+1}: {max_sh_el_deg:.2f}")
                self.max_sh_el_array.append(max_sh_el_deg)

                # Find movement start and end times
                vel_threshold = 100     # deg/s
                vel_tolerance = 10
                movement_start = None
                movement_end = None
                for t, a, v in angle_trace:
                    if v > vel_threshold and movement_start is None:
                        movement_start = t
                    if (a == max_sh_el_deg or v + vel_tolerance < 0) and movement_start is not None:
                        movement_end = t
                        break
                if movement_start and movement_end:
                    movement_duration = movement_end - movement_start
                    print(f"Movement duration for rep {rep+1}: {movement_duration:.3f} seconds")
                    self.duration_array.append(movement_duration)
                else:
                    print(f"Could not determine movement duration for rep {rep+1}")
                        
            final_max_angle = np.mean(self.max_sh_el_array)
            final_duration = np.mean(self.duration_array)
            mean_velocity = final_max_angle / final_duration
            utils.save_to_json(self.filename, round(final_max_angle, 3), "max_angle (deg)")
            utils.save_to_json(self.filename, round(final_duration, 3), "movement_duration")
            utils.save_to_json(self.filename, round(mean_velocity, 3), "mean_velocity")
            
        print("Calibration complete.")

if __name__ == "__main__":

    imu1 = Imu(IMU_RECEIVE_PORTS[0], IMU_IP_ADDRESSES[0], IMU_SEND_PORT, IMU_AXIS_UP)
    imu2 = Imu(IMU_RECEIVE_PORTS[1], IMU_IP_ADDRESSES[1], IMU_SEND_PORT, IMU_AXIS_UP)
    
    user = input("Your name: ").lower().strip()
    muscle = input("Do you want to stimulate anterior(a) or middle(m) deltoid? ").lower().strip()
    if muscle == "a":
        filename = f"{user}_anterior_calibration_data.json"
    elif muscle == "m":
        filename = f"{user}_middle_calibration_data.json"
    else:
        print("Invalid input. Exiting.")
        exit()

    mode = input("Do you want to run full calibration (f) or anti-drift only (p)? ").lower().strip()
    anti_drift = (mode == "p")

    readImu1Thread = readImuLoop("Read IMU", imu1, filename, contralateral=False, pre_calibration_only=anti_drift)
    readImu2Thread = readImuLoop("Read IMU", imu2, filename, contralateral=True)  
    readImu1Thread.start()
    readImu2Thread.start()
#!/usr/bin/python3

import threading
import time
import numpy as np
import csv

from rehamove import *

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
    sh_el_deg = 0
    max_sh_el = 0
    max_sh_el_deg = 0

    stim_current = 0

def compute_joint_angles(UA_mat):
    # Get the current shoulder elevation angle
    sh_el = np.arccos(UA_mat[2,1]) #element z of y axis
    return sh_el

class readImuLoop(threading.Thread):
    def __init__(self, name, system_state, imu, filename, calibration_mode=False, duration=10, start_event=None, max_reached=None):
        threading.Thread.__init__(self)
        self.name = name
        self.system_state = system_state
        self.imu = imu
        self.imu_fs = 200 # Need to read faster than IMU update frequency
        self.filename = filename
        self.calibration_mode = calibration_mode

        # For calibration
        self.duration = duration
        # For system control
        self.min_sh_el = np.degrees(np.pi/12)
        self.sh_el_ref = utils.load_from_json(self.filename, "max_angle")
        self.start_event = start_event
        self.max_reached = max_reached
        self.arm_lowered = True

        # IMU log file
        self.log_file = "imu_log.csv"
        # Clear previous log file content
        with open(self.log_file, "w", newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["Time", "Shoulder Elevation (deg)"])

    def run(self):
        print("Starting IMU reading thread...")
        
        dt = 1.0 / self.imu_fs
        self.imu.initialize(dt)
        self.imu.identify() # Strobe IMU leds to identify it
        IMU_mat = np.matrix([[1,0,0],[0,1,0],[0,0,1]])

        start_time = time.time()
        
        while True:

            #if self.calibration_mode and time.time() - start_time > self.duration:
            #    break  # Stop after duration if in calibration mode

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
                
                #if self.calibration_mode:
                #    self.system_state.max_sh_el = max(self.system_state.max_sh_el, self.system_state.sh_el)
              
            sh_el_deg = np.degrees(self.system_state.sh_el)
            with lock:
                self.system_state.sh_el_deg = sh_el_deg
            # Write IMU readings to log file instead of printing
            with open(self.log_file, "a", newline='') as f:
                writer = csv.writer(f)
                timestamp = time.time()
                writer.writerow([timestamp, sh_el_deg])

            # For system control, record start and stop events for stimulation
            if not self.calibration_mode and self.start_event is not None and self.max_reached is not None:
                # Trigger start event when the angle exceeds the threshold
                if sh_el_deg >= self.min_sh_el and not self.start_event.is_set() and self.arm_lowered:
                    print(f"Threshold angle {self.min_sh_el:.2f}° reached. Starting stimulation.")
                    self.start_event.set()
                    self.arm_lowered = False

                # Trigger stop event when the max angle is reached
                if sh_el_deg >= self.sh_el_ref and not self.max_reached.is_set():
                    print(f"Max angle {self.sh_el_ref:.2f}° reached. Stopping stimulation.")
                    self.max_reached.set()
                    self.start_event.clear()

                if sh_el_deg <= self.min_sh_el and self.max_reached.is_set():
                    print(f"Arm has lowered. Ready to restart stimulation.")
                    self.arm_lowered = True
                    self.max_reached.clear()

            time.sleep(max(next_time_instant - time.perf_counter(), 0))

        #if self.calibration_mode:
        #    max_sh_el_deg = np.degrees(self.system_state.max_sh_el)
        #    max_sh_el_deg = np.minimum(max_sh_el_deg, 130.0) # Cap at 130 to avoid singularity at 170 degrees (by Elena)
        #    
        #    utils.save_to_json(self.filename, max_sh_el_deg, "max_angle")
        #    
        #    print("Calibration complete. Max angle is:", max_sh_el_deg)

class FESControl(threading.Thread):
    def __init__(self, name, system_state, port_name, channel, filename, start_event, max_reached):
        threading.Thread.__init__(self)
        self.name = name
        self.system_state = system_state
        self.channel = channel
        self.filename = filename
        self.device = Rehamove(port_name)
        self.freq = 40
        self.period = 1/self.freq * 1000
        self.duration = 0.5
        self.pw = 400
        self.min_current = utils.load_from_json(self.filename, "movement_current")
        self.max_current = utils.load_from_json(self.filename, "full_range_current")
        self.start_event = start_event
        self.max_reached = max_reached

    def run(self):
        self.device.change_mode(1)
        # Waits for start event, stimulates and stops when stop event is set
        current = self.min_current

        while True:
            self.start_event.wait()
            print("Stimulation started")

            while not self.max_reached.is_set() and current <= self.max_current:
                try:
                    self.device.set_pulse(current, self.pw)
                    self.device.start(self.channel, self.period)
                    time.sleep(self.duration)
                    self.device.update()
                    with lock:
                        self.system_state.stim_current = current
                    current += 0.5  # Ramp
                except Exception as e:
                    print(f"Error during stimulation: {e}")
                    break
            
            # if max current is reached, even without reaching the angle, act as if the angle was reached (allow to restart)
            if current > self.max_current:
                self.max_reached.set() 
                self.start_event.clear()

            self.device.end()
            print("Stimulation stopped")
            current = self.min_current
            with lock:
                        self.system_state.stim_current = 0

class saveDataLoop(threading.Thread):
    def __init__(self, name, sys_state):
        threading.Thread.__init__(self)
        self.name = name
        self.sys_state = sys_state
        self.save_fs = 40
    
    def run(self):
        t0 = time.perf_counter()

        with open('log.csv', 'w') as log:
        
            file_header = "time,sh_el_deg,sh_el,stim_curr\n"
            log.write(file_header)

            while True:
                next_time_instant = time.perf_counter() + (1.0 / self.save_fs)
                t = time.perf_counter() - t0

                with lock:
                    data = "{:.5f},{:.3f},{:.3f},{:.3f}\n".format(t,self.sys_state.sh_el_deg,self.sys_state.sh_el,self.sys_state.stim_current)
                log.write(data)

                time.sleep(max(next_time_instant-time.perf_counter(),0))

def main():
    port_name = "COM7" # Windows
    #port_name = "/dev/ttyUSB0" # Linux
    possible_muscles = ["a", "m"]
    
    user = input("Your name: ").lower().strip()
    muscle = input("Do you want to stimulate anterior(a) or middle(m) deltoid? ").lower().strip()
   
    if muscle == "a":
        filename = f"{user}_anterior_calibration_data.json"
        channel = "white"
    elif muscle == "m":
        filename = f"{user}_middle_calibration_data.json"
        channel = "black"

    system_state = systemState()
    imu = Imu(IMU_RECEIVE_PORTS, IMU_IP_ADDRESSES, IMU_SEND_PORT, IMU_AXIS_UP)
    start_event = threading.Event()
    max_reached = threading.Event()
    
    readImuThread = readImuLoop("Read IMU", system_state, imu, filename, False, None, start_event, max_reached)
    stimulationThread = FESControl("Stimulation", system_state, port_name, channel, filename, start_event, max_reached)
    saveDataThread = saveDataLoop("Save data", system_state)
    
    threads = []
    threads.append(readImuThread)
    threads.append(stimulationThread)
    threads.append(saveDataThread)

    for t in threads:
        t.start()

    for t in threads:
        t.join()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("Error: ", e)
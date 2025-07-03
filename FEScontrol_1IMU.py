#!/usr/bin/python3

import threading
import time
import datetime
import numpy as np
from shared_memory import shared_memory  # for Python 3.7

from rehamove import *

import utils
from beta_function import beta_function

# Thread Lock
lock = threading.Lock()

class systemState():
    UA_mat = np.matrix([[1,0,0],[0,1,0],[0,0,1]])
    sh_el = 0
    sh_el_deg = 0
    old_sh_el_deg = 0
    curr_max_sh_el = 0
    stim_current = 0
    sh_el_error = 0

def get_latest_imu_matrix():
    shm = shared_memory.SharedMemory(name='imu_matrix')
    np_array = np.ndarray((9,), dtype=np.float64, buffer=shm.buf)
    mat = np_array.copy()  # Copy to avoid race conditions
    shm.close()
    return mat.reshape((3,3))

def compute_joint_angles(UA_mat):
    # Get the current shoulder elevation angle
    sh_el = np.arccos(UA_mat[2,1]) #element z of y axis
    return sh_el

class readImuLoop(threading.Thread):
    def __init__(self, name, system_state, emergency_stop, filename):
        threading.Thread.__init__(self)
        self.name = name
        self.system_state = system_state
        self.imu_fs = 200
        self.filename = filename
        self.precalibration_angle = utils.load_from_json(self.filename, "precalibration_angle (rad)")

        self.emergency_stop = emergency_stop

    def run(self):
        print("Starting IMU reading thread...")
        
        dt = 1.0 / self.imu_fs
        IMU_mat = np.matrix([[1,0,0],[0,1,0],[0,0,1]])
        curr_max_sh_el = 0

        while not self.emergency_stop.is_set():
            next_time_instant = time.perf_counter() + dt
            
            # Get the IMU rotation matrix
            try:
                IMU_mat = get_latest_imu_matrix()
            except Exception as e:
                print("IMU read error:", e)
                pass
            
            old_sh_el_deg = self.system_state.sh_el_deg
            sh_el = compute_joint_angles(IMU_mat) - self.precalibration_angle
            sh_el_deg = np.degrees(sh_el)
            curr_max_sh_el = max(self.system_state.curr_max_sh_el, sh_el_deg)
            
            with lock:
                self.system_state.UA_mat = IMU_mat
                self.system_state.sh_el = sh_el
                self.system_state.sh_el_deg = sh_el_deg
                self.system_state.curr_max_sh_el = curr_max_sh_el
                self.system_state.old_sh_el_deg = old_sh_el_deg
            
            time.sleep(max(next_time_instant - time.perf_counter(), 0))

class handleEvents(threading.Thread):
    def __init__(self, name, system_state, emergency_stop, filename, start_event, max_reached):
        threading.Thread.__init__(self)
        self.name = name
        self.system_state = system_state
        self.filename = filename
        self.fs = 250

        self.min_sh_el = 10
        self.sh_el_ref = utils.load_from_json(self.filename, "max_angle (deg)")
        self.start_event = start_event
        self.max_reached = max_reached

        self.emergency_stop = emergency_stop
    
    def run(self):
        dt = 1.0 / self.fs
        arm_lowered = True

        while not self.emergency_stop.is_set():
            next_time_instant = time.perf_counter() + dt
            
            #print(f"[DEBUG] sh_el_deg: {self.system_state.sh_el_deg:.2f}, start_event: {self.start_event.is_set()}, arm_lowered: {arm_lowered}, max_reached: {self.max_reached.is_set()}")

            # For system control, record start and stop events for stimulation
            # Trigger start event when the angle exceeds the threshold (and rising)
            if (self.system_state.sh_el_deg >= self.min_sh_el and 
                self.system_state.sh_el_deg > self.system_state.old_sh_el_deg and
                not self.start_event.is_set() and arm_lowered):
                print(f"Threshold angle {self.min_sh_el:.2f}° reached. Starting stimulation.")
                self.start_event.set()
                arm_lowered = False

            # When the arm is below the threshold (and lowering), update error and allow for restart
            if (self.system_state.sh_el_deg < self.min_sh_el and 
                self.system_state.sh_el_deg < self.system_state.old_sh_el_deg and
                self.max_reached.is_set()):
                print(f"Arm has lowered. Max angle for iteration: {self.system_state.curr_max_sh_el:.2f}°")

                arm_lowered = True                
                self.max_reached.clear()

                iteration_max_sh_el = self.system_state.curr_max_sh_el 
                sh_el_error = self.sh_el_ref - iteration_max_sh_el
                with lock:
                    self.system_state.sh_el_error = sh_el_error
                    self.system_state.curr_max_sh_el = 0

            time.sleep(max(next_time_instant - time.perf_counter(), 0))

class FESControl(threading.Thread):
    def __init__(self, name, system_state, emergency_stop, port_name, channel, filename, start_event, max_reached):
        threading.Thread.__init__(self)
        self.name = name
        self.system_state = system_state
        self.channel = channel
        self.filename = filename
        self.device = Rehamove(port_name)
        self.freq = 30
        self.period_ms = 1/self.freq * 1000
        self.period_s = 1/self.freq
        self.pw = 400
        self.min_current = utils.load_from_json(self.filename, "movement_current")
        self.pain_current = utils.load_from_json(self.filename, "pain_current")
        self.max_current = 0.7 * self.pain_current
        self.start_event = start_event
        self.max_reached = max_reached
        self.movement_duration = utils.load_from_json(self.filename, "movement_duration")
        mean_velocity = utils.load_from_json(self.filename, "mean_velocity")
        self.delta_t = 10 / mean_velocity
        self.T = self.movement_duration - 2 * self.delta_t
        
        self.emergency_stop = emergency_stop

    def run(self):
        # Waits for start event, stimulates and stops when stop event is set

        while not self.emergency_stop.is_set():
            if not self.start_event.wait(timeout=0.1):  # timeout to check for emergency stop
                continue
            if self.emergency_stop.is_set():
                break
            self.start_event.wait()
            print("Stimulation started")
            current = self.min_current

            sh_el_error = self.system_state.sh_el_error
            self.max_current = self.max_current + 0.05 * sh_el_error
            self.max_current = round(self.max_current*2) / 2
            if self.max_current > self.pain_current:
                self.max_current = self.pain_current - 0.5
            if self.max_current < self.min_current:
                self.max_current = self.min_current

            start_time = time.perf_counter()
            t = 0

            while not self.emergency_stop.is_set() and current <= self.max_current and t < (self.movement_duration - self.delta_t):
                next_time_instant = time.perf_counter() + self.period_s
                t = time.perf_counter() - start_time
                print("time t:", t)
                if t <= self.T:     # beta function with duration T = movement duration - 2*time it takes to get to 10°
                    i = beta_function(self.min_current, self.max_current, self.T, t) # theoretical current (continuous function)
                    current = round(i * 2 + 1e-9) / 2 # Add a small bias to ensure rounding up for ties
                elif t > self.T:     # the last ms (time it takes to get to 10°) we give constant max current
                    i = self.max_current
                
                try:
                    self.device.pulse(self.channel, current, self.pw)
                    time.sleep(max(next_time_instant-time.perf_counter(),0))
                except Exception as e:
                    print(f"Error during stimulation: {e}")
                    break

                with lock:
                        self.system_state.stim_current = current
            
            # Allow to restart
            self.max_reached.set() 
            self.start_event.clear()

            print("Stimulation stopped")
            with lock:
                self.system_state.stim_current = 0

class saveDataLoop(threading.Thread):
    def __init__(self, name, sys_state, emergency_stop, user, muscle):
        threading.Thread.__init__(self)
        self.name = name
        self.sys_state = sys_state
        self.user = user
        self.muscle = muscle.upper()
        self.date = datetime.datetime.now().strftime("%d%m")
        self.save_fs = 100

        self.emergency_stop = emergency_stop
    
    def run(self):
        dt = 1.0 / self.save_fs
        t0 = time.perf_counter()

        #filename = f"log_{self.user}_{self.muscle}{self.date}.csv" # for tests
        filename = "log.csv" # for development
        with open(filename, 'w') as log:
        
            file_header = "time,old_sh_el_deg,sh_el_deg,stim_curr\n"
            log.write(file_header)

            while not self.emergency_stop.is_set():
                next_time_instant = time.perf_counter() + dt
                t = time.perf_counter() - t0

                with lock:
                    data = "{:.5f},{:.3f},{:.3f},{:.2f}\n".format(t,self.sys_state.old_sh_el_deg,self.sys_state.sh_el_deg,self.sys_state.stim_current)
                log.write(data)

                time.sleep(max(next_time_instant-time.perf_counter(),0))

def main():
    #port_name = "COM7" # Windows
    port_name = "/dev/ttyUSB0" # Linux
    
    user = input("Your name: ").lower().strip()
    muscle = input("Do you want to stimulate anterior(a) or middle(m) deltoid? ").lower().strip()
   
    if muscle == "a":
        filename = f"{user}_anterior_calibration_data.json"
        channel = "white"
    elif muscle == "m":
        filename = f"{user}_middle_calibration_data.json"
        channel = "blue"

    system_state = systemState()
    start_event = threading.Event()
    max_reached = threading.Event()
    emergency_stop = threading.Event()
    
    readImuThread = readImuLoop("Read IMU", system_state, emergency_stop, filename, start_event, max_reached)
    handleEventsThread = handleEvents("Events", system_state, emergency_stop, filename, start_event, max_reached)
    stimulationThread = FESControl("Stimulation", system_state, emergency_stop, port_name, channel, filename, start_event, max_reached)
    saveDataThread = saveDataLoop("Save data", system_state, emergency_stop, user, muscle)
    
    threads = []
    threads.append(readImuThread)
    threads.append(handleEventsThread)
    threads.append(stimulationThread)
    threads.append(saveDataThread)

    for t in threads:
        t.start()
        
    try:
        while not emergency_stop.is_set():
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nEMERGENCY STOP TRIGGERED!")
        emergency_stop.set()

    for t in threads:
        t.join()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("Error: ", e)
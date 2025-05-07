#!/usr/bin/python3

import threading
import time
import datetime
import numpy as np
import math
import keyboard

from rehamove import *

import utils
from tdu import Imu
from beta_function import beta_function

# IMU parameters from NGIMU GUI
IMU_AXIS_UP = 'Y'
IMU_RECEIVE_PORTS = [8102, 8101]
IMU_SEND_PORT = 9000
IMU_IP_ADDRESSES = ["192.168.1.3","192.168.0.101"] # in AP mode

# Thread Lock
lock = threading.Lock()

class systemState():
    UA_mat = np.matrix([[1,0,0],[0,1,0],[0,0,1]])
    sh_el = 0
    sh_el_deg = 0
    curr_max_sh_el = 0
    stim_current = 0
    sh_el_error = 0

def compute_joint_angles(UA_mat):
    # Get the current shoulder elevation angle
    sh_el = np.arccos(UA_mat[2,1]) #element z of y axis
    return sh_el

class readImuLoop(threading.Thread):
    def __init__(self, name, system_state, emergency_stop, imu, filename, contralateral):
        threading.Thread.__init__(self)
        self.name = name
        self.system_state = system_state
        self.imu = imu
        self.imu_fs = 200 # Need to read faster than IMU update frequency
        self.filename = filename
        self.contralateral = contralateral
        self.precalibration_angle = utils.load_from_json(self.filename, "precalibration_angle (rad)")
        self.contralateral_precalibration_angle = utils.load_from_json(self.filename, "contralateral_precalibration_angle (rad)")
        '''
        self.min_sh_el = np.degrees(np.pi/12)
        self.sh_el_ref = utils.load_from_json(self.filename, "max_angle (deg)")
        self.start_event = start_event
        self.max_reached = max_reached
        self.arm_lowered = True
        '''
        self.emergency_stop = emergency_stop

    def run(self):
        print("Starting IMU reading thread...")
        
        dt = 1.0 / self.imu_fs
        self.imu.initialize(dt)
        self.imu.identify() # Strobe IMU leds to identify it
        IMU_mat = np.matrix([[1,0,0],[0,1,0],[0,0,1]])

        while not self.emergency_stop.is_set():
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
            
            if not self.contralateral:
                sh_el = compute_joint_angles(IMU_mat) - self.precalibration_angle
                sh_el_deg = np.degrees(sh_el)
                curr_max_sh_el = max(self.system_state.curr_max_sh_el, sh_el_deg)
                
                with lock:
                    self.system_state.UA_mat = IMU_mat
                    self.system_state.sh_el = sh_el
                    self.system_state.sh_el_deg = sh_el_deg
                    self.system_state.curr_max_sh_el = curr_max_sh_el

            elif self.contralateral:
                contralateral_sh_el = compute_joint_angles(IMU_mat) - self.contralateral_precalibration_angle
                contralateral_sh_el_deg = np.degrees(contralateral_sh_el)
                with lock:
                    self.system_state.contralateral_sh_el_deg = contralateral_sh_el_deg

            time.sleep(max(next_time_instant - time.perf_counter(), 0))

class handleEvents(threading.Thread):
    def __init__(self, name, system_state, emergency_stop, filename, start_event, max_reached):
        threading.Thread.__init__(self)
        self.name = name
        self.system_state = system_state
        self.filename = filename
        self.fs = 250 # same as imu fs

        self.min_sh_el = np.degrees(np.pi/12)
        self.sh_el_ref = utils.load_from_json(self.filename, "max_angle (deg)")
        self.start_event = start_event
        self.max_reached = max_reached
        self.arm_lowered = True

        self.emergency_stop = emergency_stop

    def run(self):
        dt = 1.0 / self.fs

        while not self.emergency_stop.is_set():
            next_time_instant = time.perf_counter() + dt

            # For system control, record start and stop events for stimulation
            # Trigger start event when the angle of contralateral arm exceeds the threshold
            if self.system_state.contralateral_sh_el_deg >= self.min_sh_el and not self.start_event.is_set() and self.arm_lowered:
                print(f"Threshold angle {self.min_sh_el:.2f}° reached. Starting stimulation.")
                self.start_event.set()
                self.arm_lowered = False
            '''
            # Trigger stop event when the max angle is reached
            if sh_el_deg >= self.sh_el_ref and not self.max_reached.is_set():
                print(f"Max angle {self.sh_el_ref:.2f}° reached. Stopping stimulation.")
                self.max_reached.set()
                self.start_event.clear()
            '''
            if self.system_state.sh_el_deg <= self.min_sh_el and self.max_reached.is_set():
                print(f"Arm has lowered. Max angle for iteration: {self.system_state.curr_max_sh_el:.2f}°")
                self.arm_lowered = True
                self.max_reached.clear()
                iteration_max_sh_el = self.system_state.curr_max_sh_el 
                sh_el_error = self.sh_el_ref - iteration_max_sh_el
                with lock:
                    self.system_state.sh_el_error = sh_el_error
                    self.system_state.curr_max_sh_el = 0
                # Ricorda: in questo if l'iterazione non è davvero finita, ma siamo tornati sotto pi/12 (l'angolo max salvato sarà più alto di quello "vero")
                # è da risolvere o possiamo ignorare la cosa??

            time.sleep(max(next_time_instant - time.perf_counter(), 0))

class FESControl(threading.Thread):
    def __init__(self, name, system_state, emergency_stop, port_name, channel, filename, start_event, max_reached):
        threading.Thread.__init__(self)
        self.name = name
        self.system_state = system_state
        self.channel = channel
        self.filename = filename
        self.device = Rehamove(port_name)
        self.freq = 40
        self.period_ms = 1/self.freq * 1000
        self.period_s = 1/self.freq
        self.pw = 400
        self.tingle_current = utils.load_from_json(self.filename, "tingling_current")
        self.min_current = utils.load_from_json(self.filename, "movement_current")
        self.pain_current = utils.load_from_json(self.filename, "pain_current")
        self.fullrange_current = utils.load_from_json(self.filename, "full_range_current")
        self.max_current = 0.5 * self.fullrange_current
        self.start_event = start_event
        self.max_reached = max_reached
        self.T = 2 # duration of the movement

        self.emergency_stop = emergency_stop

    def run(self):
        self.device.change_mode(1)
        # Waits for start event, stimulates and stops when stop event is set

        while not self.emergency_stop.is_set():
            if not self.start_event.wait(timeout=0.1):  # timeout to check for emergency stop
                continue
            if self.emergency_stop.is_set():
                break
            self.start_event.wait()
            print("Stimulation started")
            current = self.tingle_current

            sh_el_error = self.system_state.sh_el_error
            self.max_current = self.max_current + 0.1 * sh_el_error
            self.max_current = round(self.max_current*2) / 2
            if self.max_current > self.pain_current:
                self.max_current = self.pain_current - 0.5 # con pain_current o fullrange_current??
            if self.max_current < self.tingle_current:
                self.max_current = self.tingle_current

            start_time = time.perf_counter()
            t = 0

            while not self.emergency_stop.is_set() and current <= self.max_current and t < self.T:
                next_time_instant = time.perf_counter() + self.period_s
                t = time.perf_counter() - start_time
                print("time t:", t)
                i = beta_function(self.tingle_current, self.max_current, self.T ,t) # theoretical current (continuous function)
                current = round(i * 2 + 1e-9) / 2 # Add a small bias to ensure rounding up for ties
                
                try:
                    self.device.set_pulse(current, self.pw)
                    self.device.start(self.channel, self.period_ms)
                    self.device.update()
                    time.sleep(max(next_time_instant-time.perf_counter(),0))
               
                except Exception as e:
                    print(f"Error during stimulation: {e}")
                    break

                with lock:
                        self.system_state.stim_current = current
            
            # if max current is reached, even without reaching the angle, allow to restart
            if current >= self.max_current:
                print("Max current reached. Stopping stimulation")
                self.max_reached.set() 
                self.start_event.clear()

            self.device.end()
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
        
            file_header = "time,sh_el_deg,sh_el,stim_curr,max_sh_el(deg)\n"
            log.write(file_header)

            while not self.emergency_stop.is_set():
                next_time_instant = time.perf_counter() + dt
                t = time.perf_counter() - t0

                with lock:
                    data = "{:.5f},{:.3f},{:.3f},{:.2f},{:.3f}\n".format(t,self.sys_state.sh_el_deg,self.sys_state.sh_el,self.sys_state.stim_current,self.sys_state.curr_max_sh_el)
                log.write(data)

                time.sleep(max(next_time_instant-time.perf_counter(),0))

def main():
    port_name = "COM7" # Windows
    #port_name = "/dev/ttyUSB0" # Linux
    
    user = input("Your name: ").lower().strip()
    muscle = input("Do you want to stimulate anterior(a) or middle(m) deltoid? ").lower().strip()
   
    if muscle == "a":
        filename = f"{user}_anterior_calibration_data.json"
        channel = "white"
    elif muscle == "m":
        filename = f"{user}_middle_calibration_data.json"
        channel = "black"

    system_state = systemState()
    imu1 = Imu(IMU_RECEIVE_PORTS[0], IMU_IP_ADDRESSES[0], IMU_SEND_PORT, IMU_AXIS_UP)
    imu2 = Imu(IMU_RECEIVE_PORTS[1], IMU_IP_ADDRESSES[1], IMU_SEND_PORT, IMU_AXIS_UP)
    start_event = threading.Event()
    max_reached = threading.Event()
    emergency_stop = threading.Event()
    
    readImu1Thread = readImuLoop("Read IMU", system_state, emergency_stop, imu1, filename, contralateral=False)
    readImu2Thread = readImuLoop("Read IMU", system_state, emergency_stop, imu2, filename, contralateral=True)
    handleEventsThread = handleEvents("Events", system_state, emergency_stop, filename, start_event, max_reached)
    stimulationThread = FESControl("Stimulation", system_state, emergency_stop, port_name, channel, filename, start_event, max_reached)
    saveDataThread = saveDataLoop("Save data", system_state, emergency_stop, user, muscle)
    
    threads = []
    threads.append(readImu1Thread)
    threads.append(readImu2Thread)
    threads.append(handleEventsThread)
    threads.append(stimulationThread)
    threads.append(saveDataThread)

    for t in threads:
        t.start()

    while not emergency_stop.is_set():
        if keyboard.is_pressed('esc'):
            print("\nEMERGENCY STOP TRIGGERED!")
            emergency_stop.set()
            break
        time.sleep(0.1)

    for t in threads:
        t.join()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("Error: ", e)
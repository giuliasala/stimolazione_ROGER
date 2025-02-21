import threading
import time
import numpy as np

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
    max_sh_el = 0

def compute_joint_angles(UA_mat):
    # Get the current shoulder elevation angle
    sh_el = np.arccos(UA_mat[2,1]) #element z of y axis
    return sh_el

class readImuLoop(threading.Thread):
    def __init__(self, name, system_state, imu, filename, calibration_mode=False, duration=10, start_event=None, stop_event=None):
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
        self.min_sh_el = np.pi/12
        self.max_sh_el = utils.load_from_json(self.filename, "max_angle")
        self.start_event = start_event
        self.stop_event = stop_event

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

            # For system control, record start and stop events for stimulation
            if not self.calibration_mode and self.start_event is not None and self.stop_event is not None:
                # Trigger start event when the angle exceeds the threshold
                if self.start_event and sh_el_deg >= self.min_sh_el and not self.start_event.is_set():
                    print(f"Threshold angle {self.min_sh_el:.2f}° reached. Starting stimulation.")
                    self.start_event.set()

                # Trigger stop event when the max angle is reached
                if self.stop_event and sh_el_deg >= self.max_sh_el and not self.stop_event.is_set():
                    print(f"Max angle {self.max_sh_el:.2f}° reached. Stopping stimulation.")
                    self.stop_event.set()

            time.sleep(max(next_time_instant - time.perf_counter(), 0))

        if self.calibration_mode:
            max_sh_el_deg = np.degrees(self.system_state.max_sh_el)
            max_sh_el_deg = np.minimum(max_sh_el_deg, 130.0) # Cap at 130 to avoid singularity at 170 degrees (by Elena)
            
            utils.save_to_json(self.filename, max_sh_el_deg, "max_angle")
            
            print("Calibration complete. Max angle is:", max_sh_el_deg)
        
        print("IMU thread finished.")

class FESControl(threading.Thread):
    def __init__(self, name, system_state, port_name, channel, filename, start_event, stop_event):
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
        self.current = utils.load_from_json(self.filename, "movement_current")
        self.max_current = utils.load_from_json(self.filename, "full_range_current")
        self.min_sh_el = np.pi/12
        self.max_sh_el = utils.load_from_json(self.filename, "max_angle")
        self.start_event = start_event
        self.stop_event = stop_event

    def run(self):
        # Waits for start event, stimulates and stops when stop event is set
        self.start_event.wait()
        print("Stimulation started")

        while not self.stop_event.is_set():
            try:
                self.device.set_pulse(self.current, self.pw)
                self.device.start(self.channel, self.period)
                time.sleep(self.duration)
                self.device.update()
                self.current += 0.5  # Ramp
            except Exception as e:
                print(f"Error during stimulation: {e}")
                break

        self.device.end()
        print("Stimulation stopped")

def main():
    port_name = "COM7"
    possible_muscles = ["a", "m"]
    
    muscle = input("Do you want to stimulate anterior(a) or middle(m) deltoid? ").lower().strip()
   
    if muscle == "a":
        filename = "anterior_calibration_data.json"
        channel = "white"
    elif muscle == "m":
        filename = "middle_calibration_data.json"
        channel = "black"

    calibration_mode = input("Start calibration? (y/n) ").lower().strip()

    system_state = systemState()
    imu = Imu(IMU_RECEIVE_PORTS, IMU_IP_ADDRESSES, IMU_SEND_PORT, IMU_AXIS_UP)
    start_event = threading.Event()
    stop_event = threading.Event()

    if calibration_mode == "y":
        duration_str = input("Duration (number): ")
        duration = int(duration_str)

        readImuThread = readImuLoop("Read IMU", system_state, imu, filename, True, duration)
    
    else:
        readImuThread = readImuLoop("Read IMU", system_state, imu, filename, False, None, start_event, stop_event)

    stimulationThread = FESControl("Stimulation", system_state, port_name, channel, filename, start_event, stop_event)
    
    readImuThread.start()
    stimulationThread.start()

    readImuThread.join()
    stimulationThread.join()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("Error: ", e)
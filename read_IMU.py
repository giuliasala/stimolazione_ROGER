from FEATHER_bbai_main.tdu import Imu
import numpy as np
import threading
import time

# IMU parameters from NGIMU GUI
IMU_AXIS_UP = 'Y'
IMU_RECEIVE_PORTS = 8101
IMU_SEND_PORT = 9000
IMU_IP_ADDRESSES = "192.168.1.1" # in AP mode

# Thread Lock
lock = threading.Lock()

class systemState():
    UA_mat = np.matrix([[1,0,0],[0,1,0],[0,0,1]])
    sh_el = 0

def compute_joint_angles(UA_mat):
    # Get the current shoulder elevation angle
    sh_el = np.arccos(UA_mat[2,1]) #element z of y axis
    return sh_el

class readImuLoop(threading.Thread):
    def __init__(self, name, system_state, imu, segment):
        threading.Thread.__init__(self)
        self.name = name
        self.system_state = system_state
        self.imu = imu
        self.imu_fs = 200 # Need to read faster than IMU update frequency
        self.segment = segment

    def run(self):
        print("Starting IMU reading thread...")
        
        dt = 1.0 / self.imu_fs
        self.imu.initialize(dt)
        self.imu.identify() # Strobe IMU leds to identify it
        IMU_mat = np.matrix([[1,0,0],[0,1,0],[0,0,1]])
        
        while True:
            next_time_instant = time.perf_counter() + dt
            # Get the IMUs rotation matrices
            try:
                IMU_m = self.imu.read_imu()
                IMU_mat = np.matrix([[IMU_m[0],IMU_m[1],IMU_m[2]],[IMU_m[3],IMU_m[4],IMU_m[5]],[IMU_m[6],IMU_m[7],IMU_m[8]]])
            except Exception as e:
                print("IMU read error:", e)
                pass
                
            with lock:
                if self.segment == "ua":
                    self.system_state.UA_mat = IMU_mat
                    self.system_state.sh_el = compute_joint_angles(IMU_mat)
                elif self.segment == "fa":
                    self.system_state.FA_mat = IMU_mat
              
            time.sleep(max(next_time_instant-time.perf_counter(),0))
            print(f"Shoulder Elevation (deg): {np.degrees(self.system_state.sh_el):.2f}")

def main():        
    system_state = systemState()

    imu = Imu(IMU_RECEIVE_PORTS, IMU_IP_ADDRESSES, IMU_SEND_PORT, IMU_AXIS_UP)

    readImuThread = readImuLoop("Read IMU", system_state, imu, "ua")
    readImuThread.start()

if __name__ == "__main__":
    main()
#!/usr/bin/python3

import numpy as np
import time
from shared_memory import shared_memory  # for Python 3.7

from tdu import Imu

# IMU parameters from NGIMU GUI
IMU_AXIS_UP = 'Y'
IMU_RECEIVE_PORT = 8101
IMU_SEND_PORT = 9000
IMU_IP_ADDRESS = "192.168.1.1"

imu_fs = 200 #Hz
imu_dt = 1 / imu_fs
# Shared memory for 9 floats (3x3 matrix)
shm = shared_memory.SharedMemory(create=True, size=9*8, name='imu_matrix')
imu = Imu(IMU_RECEIVE_PORT, IMU_IP_ADDRESS, IMU_SEND_PORT, IMU_AXIS_UP)
imu.initialize(imu_dt)
imu.identify

try:
    while True:
        IMU_m = imu.read_imu()  # Should return a list of 9 floats
        np_array = np.ndarray((9,), dtype=np.float64, buffer=shm.buf)
        np_array[:] = IMU_m
        time.sleep(imu_dt)
except KeyboardInterrupt:
    pass
finally:
    shm.close()
    shm.unlink()
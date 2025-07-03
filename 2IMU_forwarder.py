#!/usr/bin/python3

import numpy as np
import time
from shared_memory import shared_memory  # for Python 3.7

from tdu import Imu

# IMU parameters from NGIMU GUI
IMU_AXIS_UP = 'Y'
IMU_RECEIVE_PORTS = [8100, 8102]
IMU_SEND_PORT = 9000
IMU_IP_ADDRESSES = ["192.168.0.101", "192.168.0.102"]

imu_fs = 200 #Hz
imu_dt = 1 / imu_fs
# Shared memory for 9 floats (3x3 matrix)
shm = shared_memory.SharedMemory(create=True, size=9*8, name='imu_matrix')
shm_contra = shared_memory.SharedMemory(create=True, size=9*8, name='imu_matrix_contra')
imu = Imu(IMU_RECEIVE_PORTS[0], IMU_IP_ADDRESSES[0], IMU_SEND_PORT, IMU_AXIS_UP)
imu_contra = Imu(IMU_RECEIVE_PORTS[1], IMU_IP_ADDRESSES[1], IMU_SEND_PORT, IMU_AXIS_UP)
imu.initialize(imu_dt)
imu_contra.initialize(imu_dt)
imu.identify()
imu_contra.identify()

try:
    while True:
        IMU_m = imu.read_imu()  # Should return a list of 9 floats
        IMU_m_contra =imu_contra.read_imu()
        np_array = np.ndarray((9,), dtype=np.float64, buffer=shm.buf)
        np_array_contra = np.ndarray((9,), dtype=np.float64, buffer=shm_contra.buf)
        np_array[:] = IMU_m
        np_array_contra[:] = IMU_m_contra
        time.sleep(imu_dt)
except KeyboardInterrupt:
    pass
finally:
    shm.close()
    shm.unlink()
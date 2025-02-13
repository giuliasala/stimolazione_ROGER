#!/usr/bin/python3
from tdu import Motor
from tdu import MotorState
from tdu import Imu
from tdu import loadCell

import csv
import datetime
import numpy as np
import rcpy
import signal
import subprocess
import sys
import threading
import time
import warnings

from datetime import datetime

import scipy
import scipy.signal

# For CAN Bus
from can.interface import Bus
import binascii

import can
can.rc['interface'] = 'socketcan'
can.rc['channel'] = 'can0'
can.rc['bitrate'] = 1000000

import tmotor_can_utils as tcan

# Motors specs
CAN_IDs = [1,2]
MODELS = ["AK60_6", "AK60_6"]
MOTOR_JOINT_ASSIGNMENT = ["shoulder","elbow"]

print("Disabling motor!")
jj=0
motor1 = Motor(CAN_ID = 1, FS = 100.0, model = "AK60_6", joint = MOTOR_JOINT_ASSIGNMENT[0])
motor2 = Motor(CAN_ID = 2, FS = 100.0, model = "AK60_6", joint = MOTOR_JOINT_ASSIGNMENT[1])
while jj<4:
    print('Attempt'+ str(jj+1))
    motor1.enable_and_stop()
    motor1.stop_and_disable()
    motor2.enable_and_stop()
    motor2.stop_and_disable()
    jj+=1
    time.sleep(0.2)
print('M1 disabled!')  
print('M2 disabled!')

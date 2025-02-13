#!/usr/bin/python3

from tdu import Motor
from tdu import MotorState
from tdu import ControlState
from tdu import Imu
from tdu import loadCell

from pythonosc import udp_client

import csv
import datetime
import numpy as np
import math
from numpy.linalg import norm
import rcpy
import signal
import subprocess
import sys
import threading
import time
import warnings
import os

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

import json

f = open('/home/debian/FEATHER/FEATHER_bbai/settings.json')
set = json.load(f)
f.close()

# Suit geometrical parameters
PULLEY_R = 0.035

# User Data
M = set["physical_settings"][0]["weight"]
H = set["physical_settings"][0]["height"]/100

# Constants
G = np.mat([0,0,-9.81]) # acceleration of gravity
    
# Assistance percentage
ASSIST_PERC_SH = set["preference"][0]["perc_sh_assist"]
ASSIST_PERC_EL = set["preference"][0]["perc_el_assist"]

# Mass of object held
OBJ_MASS = set["preference"][0]["object_mass"]

# Supported arm
SUPPORTED_ARM = set["preference"][0]["supported_arm"]

# Elbow and shoulder support
SHOULDER_SUPPORT = set["preference"][0]["sh_assist"]
ELBOW_SUPPORT = set["preference"][0]["el_assist"]

# Minimum and Maximum cable tensions (N)
MIN_TENSION = 5.0
MAX_TENSION = 70.0

# Minimum and Maximum motor torques (Nm)
MIN_TORQUE = 0.1
MAX_TORQUE = 9.0

# Maximum motor speed (rad/s)
MAX_SPEED = 30.0

# Pretension setpoint (N)
PRETENSION = 10.0

# Main control loop frequency
FS = 100.0
DT = 1.0 / FS

# Motors specs
CAN_IDs = [1,2]
MOTOR_MODELS = ["AK60_6", "AK60_6"]

# Control parameters for position control
KP_1 = 2.0
KD_1 = 2.5

KP_2 = 2.0
KD_2 = 2.5

# Load cell parameters from calibration
LC_OFFSET = 0.074
LC_GAIN = 1/0.00545

# Parameters for friction compensation model
if set["preference"][0]["Friction_compensation"] == True:
    sh_friction_pp = np.loadtxt('/home/debian/FEATHER/FEATHER_bbai/sh_parameters.csv', delimiter=',')
    el_friction_pp = np.loadtxt('/home/debian/FEATHER/FEATHER_bbai/el_parameters.csv', delimiter=',')
else:
    sh_friction_pp = [0.0, 0.0, 0.0, 0.0, 0.0]

print(sh_friction_pp)

# IMU parameters from NGIMU GUI - The first is the upper arm, the second is the forearm
IMU_AXIS_UP = 'Y'
IMU_RECEIVE_PORTS = [8101,8102]
IMU_SEND_PORT = 9000
IMU_IP_ADDRESSES = ["192.168.1.1","192.168.0.102"] # in AP mode only for IMU 1
#IMU_IP_ADDRESSES = ["192.168.0.101","192.168.0.102"] #when in client mode the IP will look more like this

# Thread Lock
lock = threading.Lock()

class systemState():
    v_bat = 0
    
    motor_state = [MotorState(), MotorState()]
    control_state = [ControlState(), ControlState()]

    UA_mat = np.matrix([[1,0,0],[0,1,0],[0,0,1]])
    FA_mat = np.matrix([[1,0,0],[0,1,0],[0,0,1]])
    sh_el = 0
    sh_el_dot = 0   
    sh_hr = 0
    sh_hr_dot = 0
    el_fe = 0
    el_fe_dot = 0

    sh_force_des = 0
    el_force_des = 0  
    force_cur = 0
    
    trigger = 0
    
    gravity_support_shoulder = ASSIST_PERC_SH
    gravity_support_elbow = ASSIST_PERC_EL

class kalmanFilter():
    def __init__(self):
        # Define the system transition matrix
        self.F = np.array([[1., DT, 0.5 * DT**2],[0., 1., DT],[0, 0, 1.0]])

        # Define the observation matrix
        self.H = np.array([[1., 0., 0]])
        
        # Define the process noise covariance
        self.Q = np.array([[0.01, 0, 0],[0, 10, 0],[0, 0, 1000]])

        # Define the measurement noise covariance
        self.R = np.array([[0.0001]])

        # Initial state estimate
        self.x = np.array([0, 0, 0])  # Assuming initial length and speed are zero

        # Initial estimate covariance
        self.P = np.array([[1., 0, 0],[0, 1., 0], [0, 0, 1]])
              
    def update_kalman(self,z):
        # Prediction step
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

        # Measurement update step
        y = z - (self.H @ self.x)
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(3) - K @ self.H) @ self.P

        # Return position and velocity estimates
        return self.x[0], self.x[1]

def compute_joint_angles(UA_mat,FA_mat): #TO DO: make possible to choose between simple and complex
    # Get the current shoulder elevation angle and elbow flexion
    sh_el = np.arccos(UA_mat[2,1]) #element z of y axis
    el_fe = 0
    sh_hr = 0
    #el_fe = math.atan2(norm(np.cross(FA_mat[:,1].T,UA_mat[:,1].T),1),(np.dot(FA_mat[:,1].T,UA_mat[:,1])))
    #num = np.cos(sh_el)*np.sin(el_fe-np.pi/2)-FA_mat[2,1]
    #den = np.cos(el_fe-np.pi/2)*np.sin(sh_el)
    #sh_hr =  math.acos(np.clip(num/den,-1,1))-np.pi/2
    return sh_el, sh_hr, el_fe
    
def compute_gravity_assistance(UA_mat,FA_mat,sh_el,el_fe,sh_hr,el_fe_dot):

    # Cap angle at 130 degrees to avoid modelling singularity at ~170 ish degrees
    sh_el = np.minimum(sh_el, 130.0*np.pi/180.0)

    # Anthropometry
    h = 0.06 # Height of cable exit above skin
    r_ac = 0.0430 * H # Radius of rotation of GH joint (GH to acromion)
    m_upper_arm = 0.028 * M
    l_upper_arm = 0.186 * H
    l_com_upper_arm = 0.436 * l_upper_arm
    l_anchor_sh = 0.7 * l_upper_arm # Distance to anchor point along humerus
    m_forearm = 0.022 * M
    l_forearm = 0.146 * H
    l_com_forearm = 0.682 * l_forearm
    l_anchor_el = 0.7 * l_forearm # Distance to anchor point along ulna
    m_obj = OBJ_MASS
    
    #TO DO: make possible to choose between simple and complex
    ''' 
    #Complex gravity compensation
    if sh_el < np.pi/6:
        sh_hr = -np.pi/2
    elif sh_el >= np.pi/6 and el_fe < np.pi/6:
        sh_hr = 0
    
    
    G_mat = np.mat([[1,0,0],[0,0,-1],[0,1,0]])
    S_coord = np.mat([0,0,0])
    UA_adj = np.mat([[np.cos(sh_el)*np.cos(sh_hr-np.pi/2),np.sin(sh_el),-np.cos(sh_el)*np.sin(sh_hr-np.pi/2)],[np.sin(sh_hr-np.pi/2),0,np.cos(sh_hr-np.pi/2)],[np.cos(sh_hr-np.pi/2)*np.sin(sh_el),-np.cos(sh_el),0]])
    UA_com_coord = l_com_upper_arm * UA_adj[:,1].T
    FA_adj = np.mat([[np.sin(sh_el)*np.sin(el_fe-np.pi/2)+np.cos(sh_el)*np.cos(sh_hr-np.pi/2)*np.cos(el_fe-np.pi/2), np.cos(sh_el)*np.sin(sh_hr-np.pi/2), np.cos(el_fe-np.pi/2)*np.sin(sh_el)-np.cos(sh_el)*np.cos(sh_hr-np.pi/2)*np.sin(el_fe-np.pi/2)],[np.cos(el_fe-np.pi/2)*np.sin(sh_hr-np.pi/2),-np.cos(sh_hr-np.pi/2),-np.sin(sh_hr-np.pi/2)*np.sin(el_fe-np.pi/2)],[np.cos(sh_hr-np.pi/2)*np.cos(el_fe-np.pi/2)*np.sin(sh_el)-np.cos(sh_el)*np.sin(el_fe-np.pi/2),np.sin(sh_el)*np.sin(sh_hr-np.pi/2),-np.cos(sh_el)*np.cos(el_fe-np.pi/2)-np.cos(sh_hr-np.pi/2)*np.sin(sh_el)*np.sin(el_fe-np.pi/2)]])
    E_coord = l_upper_arm * UA_adj[:,1].T
    FA_com_coord = E_coord - l_com_forearm * FA_adj[:,0].T
    H_coord = E_coord - l_forearm * FA_adj[:,0].T
    # Shoulder gravity
    sh_tau = m_upper_arm * np.cross(G,UA_com_coord-S_coord) + m_forearm * np.cross(G,FA_com_coord-S_coord) + m_obj * np.cross(G,H_coord-S_coord)
    sh_tau_proj = G_mat.T * sh_tau.T
    sh_tau_req = sh_tau_proj[2,0]* ASSIST_PERC_SH
    
    '''
    
    #Simple gravity compensation considering elbow always fully extended
    sh_tau = m_upper_arm * np.abs(G[0,2]) * l_com_upper_arm + m_forearm * np.abs(G[0,2]) * (l_upper_arm + l_com_forearm) + m_obj * np.abs(G[0,2]) * (l_upper_arm + l_forearm)
    sh_tau_req = sh_tau * np.sin(sh_el) * ASSIST_PERC_SH
    
    #Geometrical model
    w = np.sqrt(l_anchor_sh**2 + (0.1*r_ac)**2)
    phi = np.arctan2(r_ac, l_anchor_sh)
    num = w - (h + r_ac) * np.cos(np.pi - sh_el - phi)
    den = np.sqrt( (h + r_ac)**2 + w**2 - 2*(h + r_ac) * w * np.cos(np.pi - sh_el - phi))
    alpha = np.arccos(num / den)   
    
    # Desired shoulder tendon tension
    sh_el_thr = np.pi/12
    sh_T_tendon_steepness = 2
    sh_T_tendon = sh_tau_req / (w * np.sin(alpha))
    sh_T_tendon = sh_T_tendon * (np.tanh((sh_el-sh_el_thr)*sh_T_tendon_steepness)+1)/2
    
    # Elbow gravity
    '''
    el_tau = m_forearm * np.cross(G,FA_com_coord-E_coord) + m_obj * np.cross(G,H_coord-E_coord)
    el_tau_proj = UA_adj.T * el_tau.T
    el_tau_req = el_tau_proj[2,0] * ASSIST_PERC_EL
    a = np.sqrt((l_upper_arm-l_anchor_sh)**2 + r_ac**2)
    b = np.sqrt((l_anchor_el)**2 + r_ac**2)
    phi2 = np.pi - el_fe - np.arctan2(r_ac,(l_upper_arm-l_anchor_sh)) - np.arctan2(r_ac,(l_anchor_el))
    l_cable = np.sqrt(a**2 + b**2 - 2*a*b*np.cos(phi2))
    eps = np.arccos((b**2 + l_cable**2 - a**2)/(2*b*l_cable))
    beta = eps -  np.arctan2(r_ac,l_anchor_el)
    '''
    el_tau_req = (-m_forearm * G[0,2] * l_com_forearm - m_obj * G[0,2] * l_forearm) * ASSIST_PERC_EL
    a = np.sqrt((l_upper_arm-l_anchor_sh)**2 + r_ac**2)
    b = np.sqrt((l_anchor_el)**2 + r_ac**2)
    phi2 = np.pi - np.pi/2 - np.arctan2(r_ac,(l_upper_arm-l_anchor_sh)) - np.arctan2(r_ac,(l_anchor_el))
    l_cable = np.sqrt(a**2 + b**2 - 2*a*b*np.cos(phi2))
    eps = np.arccos((b**2 + l_cable**2 - a**2)/(2*b*l_cable))
    beta = eps -  np.arctan2(r_ac,l_anchor_el)
    
    # Desired elbow tendon tension
    el_fe_dot_th = 0
    el_T_tendon_steepness = 1
    el_T_tendon = el_tau_req / (np.cos(beta)*r_ac + np.sin(beta)*l_anchor_el)
    el_T_tendon = el_T_tendon * (np.tanh((el_fe_dot-el_fe_dot_th)*el_T_tendon_steepness)+1)/2

    # Clip output tension (pretension and safety)
    sh_T_tendon = np.clip(sh_T_tendon, MIN_TENSION, MAX_TENSION)
    el_T_tendon = np.clip(el_T_tendon, MIN_TENSION, MAX_TENSION)
   
    return sh_T_tendon, el_T_tendon
    
def compute_friction_compensation(T_tendon_des, vel, pp):  
    
    alpha = pp[4]
    fs = pp[1]
    fc = pp[0]
    v_brk = pp[2]
    delta = pp[3]
    
    # TO DO: pass these parameters from the function because they might be different between sh and el
    joint_tdu_vel_ratio = 1.3147 # this actually depends on the anthropometry of the user --> possibly make it data-driven?
    v_thr = 0.02
    
    #sh_vel=filter(sh_el_dot_cur)
    v_cable = vel * joint_tdu_vel_ratio * PULLEY_R
    
    # Modified controller to take into account TDU-IMU speed shift. Does not need sigmoid
    if (np.abs(v_cable) <= v_thr):
        f = fs*(v_cable/v_thr)
    else:
        f = (fc + (fs-fc)*np.exp(-np.abs((v_cable-np.sign(v_cable)*v_thr)/v_brk)**delta))*np.sign(v_cable)
    
    T_in = T_tendon_des * (1+f)/(1-f)/(1-alpha)
    
    tau_des = T_in * PULLEY_R
    tau_des = np.clip(tau_des, MIN_TORQUE, MAX_TORQUE)

    return tau_des

## Define threads ##
class motorControlLoop(threading.Thread):
    def __init__(self, name, sys_state, motor, control_mode):
        threading.Thread.__init__(self)
        self.name = name
        self.sys_state = sys_state
        self.motor = motor
        self.control_mode = control_mode

    def run(self):
        self.motor.stop_and_disable()
        time.sleep(0.1)
        counter = 0

        with lock:
            can_id = self.motor.can_id
            motor_cur_pos = self.sys_state.motor_state[can_id-1].position
            print("Taring Motor {}".format(can_id))
            print("Motor {} Current Position: {}".format(can_id,motor_cur_pos))   

        while (np.abs(motor_cur_pos) > 0.1):
            self.motor.stop_and_disable()
            self.motor.tare_position()
            with lock:
                motor_cur_pos = self.sys_state.motor_state[can_id-1].position
                print("Motor {} Tared Position: {}".format(can_id,motor_cur_pos))                   
        
        # Define Pretension Details
        is_system_pretensioned = False    
        with lock:
            kp,kd = self.motor.get_control_gains()
        self.motor.set_control_mode("velocity", kp, kd)
        
        cable_pretension_setpoint = PRETENSION
        print("Pretension Setpoint: ", cable_pretension_setpoint, " N.")

        # Now finally enable motor
        self.motor.enable_and_stop()
        
        # MAIN CONTROL LOOP
        try:
            while rcpy.get_state() != rcpy.EXITING:

                next_time_instant = time.perf_counter() + DT

                with lock:
                    force_cur = self.sys_state.force_cur
                    
                    # Torque safety cutoffs
                    t_cur_filt = self.motor.update_filtered_torque(self.motor.motor_state.torque)
                    torque_cutoff_condition_m = np.abs(t_cur_filt) > self.motor.safety_limits.cutoff_torque

                    # Speed safety cutoffs
                    speed_cutoff_condition_m = np.abs(self.motor.motor_state.velocity) > self.motor.safety_limits.cutoff_speed          
                
                if (torque_cutoff_condition_m):
                    warnings.warn("Cutoff torque exceeded, system exiting...")
                    rcpy.set_state(rcpy.EXITING)
                
                if(speed_cutoff_condition_m):
                    warnings.warn("Cutoff velocity exceeded, system exiting...")
                    rcpy.set_state(rcpy.EXITING)

                # Init desired state
                p_des_m = 0
                v_des_m = 0
                t_ff_m = 0
                sh_force_des = 0
                el_force_des = 0

                # PRETENSION BLOCK (only executes once)
                if is_system_pretensioned == False:
                    v_des_m = 0.5
                    if self.motor.can_id == 1:
                        v_des_m = -v_des_m
                    t_cable_est = np.abs(t_cur_filt) / PULLEY_R 
                    #t_cable_meas = force_cur # only if load cell is present
                    t_ff_m = 0
                    self.motor.set_ref_velocity(v_des_m, t_ff_m)
                    #if self.motor.can_id == 1:
                        #print("Shoulder cable tension: ", t_cable_meas)
                    #elif self.motor.can_id == 2:
                        #print("Elbow cable tension: ", t_cable_meas)

                    if t_cable_est > cable_pretension_setpoint:
                        is_system_pretensioned = True
                        # Set motor control type: "trajectory", "torque", "velocity
                        self.motor.set_control_mode(self.control_mode, kp, kd)
                    
                        with lock:
                            self.sys_state.trigger = self.sys_state.trigger + 1
                        
                    if self.sys_state.trigger == 2:
                        # TRIGGER HIGH
                        os.popen('echo out > /sys/class/gpio/gpio105/direction')
                        os.popen('echo 1 > /sys/class/gpio/gpio105/value')
                        os.popen('echo 0 > /sys/class/gpio/gpio105/value')
                   
                else: # MAIN CONTROL LOOP AFTER PRETENSION ACHIEVE

                    if self.control_mode == "torque":
                        p_des_m = 0
                        v_des_m = 0
                                            
                        
                        if self.motor.joint == "shoulder":
                            with lock:
                                sh_force_des = self.sys_state.sh_force_des
                                sh_el_dot_cur = self.sys_state.sh_el_dot
                            t_ff_m = compute_friction_compensation(sh_force_des, sh_el_dot_cur, sh_friction_pp)
                            
                        if self.motor.joint == "elbow":
                            with lock:
                                el_force_des = self.sys_state.el_force_des
                                el_fe_dot_cur = self.sys_state.el_fe_dot
                            t_ff_m = compute_friction_compensation(el_force_des, el_fe_dot_cur, el_friction_pp)
          
                        # Reverse direction of output for left motor
                        if self.motor.can_id == 1:
                            t_ff_m = -1.0* t_ff_m

                        # Send command to motor
                        self.motor.set_ff_torque(t_ff_m)

                # Save desired state to system
                with lock:
                    self.sys_state.control_state[can_id-1] = self.motor.control_state
                
                counter = counter+1
                time.sleep(max(next_time_instant-time.perf_counter(),0))


            # Force P8_08 LOW
            os.popen('echo 0 > /sys/class/gpio/gpio105/value')
            
            with lock:
                self.sys_state.trigger = 0
            
            print("Disabling motor!")
            ii=0
            while ii<4:
                print('Attempt'+ str(ii+1))
                self.motor.stop_and_disable()
                ii+=1
                time.sleep(0.5)
            if self.motor.can_id == 1:    
                print('M1 disabled!')
            elif self.motor.can_id == 2:    
                print('M2 disabled!')
                
        except:
            print("Disabling motor!")
            ii=0
            while ii<4:
                print('Attempt'+ str(ii+1))
                self.motor.stop_and_disable()
                ii+=1
                time.sleep(0.5)
            if self.motor.can_id == 1:    
                print('M1 disabled!')
            elif self.motor.can_id == 2:    
                print('M2 disabled!')
        
            
            
class readCANBusLoop(threading.Thread):
    def __init__(self, name, sys_state, motors):
        threading.Thread.__init__(self)
        self.name = name
        self.sys_state = sys_state
        self.motors = motors
        self.can_read_fs = 2*FS # Run thread faster than main controller so that messages are always up-to-date.

        # The motors do not respond with the CAN ID in the correct arbitration field
        # but rather in the first byte of data. And the arbitration field is 0x000
        # for motor responses, so we filter for these messages (so we don't pick up
        # sent commands on the CAN bus)
        filters = [{"can_id": 0x000, "can_mask": 0x7FF, "extended": False}]
        self.bus = Bus(channel="can0", bustype="socketcan", can_filters=filters, bitrate=1000000)

    def run(self):
        print("Initializing CAN...")
        while rcpy.get_state() != rcpy.EXITING:
            next_time_instant = time.perf_counter() + (1.0 / self.can_read_fs) 
            cur_msg = self.bus.recv(timeout = 0) # TODO: What value for timeout?    
            while cur_msg is not None:
                #print(binascii.hexlify(cur_msg.data))
                can_id, position, velocity, torque, temperature, error_code = tcan.unpack_reply(cur_msg.data, CAN_IDs, MOTOR_MODELS)
              
                try:    
                    self.motors[can_id-1].motor_state.can_id = can_id
                    self.motors[can_id-1].motor_state.position = position
                    self.motors[can_id-1].motor_state.velocity = velocity
                    self.motors[can_id-1].motor_state.torque = torque
                    self.motors[can_id-1].motor_state.temperature = temperature
                    self.motors[can_id-1].motor_state.error_code = error_code
                except:
                    print("Unknown CAN ID")
                # Attempt to get the next message
                cur_msg = self.bus.recv(timeout = 0)
            
                with lock:
                    # Update system state with motor state
                    self.sys_state.motor_state[can_id-1] = self.motors[can_id-1].motor_state

            time.sleep(max(next_time_instant-time.perf_counter(),0))

class readLoadCellLoop(threading.Thread):
    def __init__(self, name, sys_state):
        threading.Thread.__init__(self)
        self.name = name
        self.sys_state = sys_state
        self.load_cell_fs = FS # Use the system frequency

    def run(self):
        print("Starting load cell reading...")
        load_cell = loadCell(LC_OFFSET, LC_GAIN)
        load_cell.initialize()       
        count = 0
        while rcpy.get_state() != rcpy.EXITING:
            next_time_instant = time.perf_counter() + (1.0 / self.load_cell_fs)

            force = load_cell.read_force()
            #volt = load_cell.read_volt()

            #Print force every 100 ticks (roughly 1 second)
            # if count%100 == 0:
            #     print("Force: ", force, "N")
            
            with lock:
                self.sys_state.force_cur = force

            count = count + 1
            time.sleep(max(next_time_instant-time.perf_counter(),0))
            
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
        
        while rcpy.get_state() != rcpy.EXITING:
            next_time_instant = time.perf_counter() + (1.0 / self.imu_fs)
            # Get the IMUs rotation matrices
            try:
                IMU_m = self.imu.read_imu()
                IMU_mat = np.matrix([[IMU_m[0],IMU_m[1],IMU_m[2]],[IMU_m[3],IMU_m[4],IMU_m[5]],[IMU_m[6],IMU_m[7],IMU_m[8]]])
            except:
                pass
                
            with lock:
                if self.segment == "ua":
                    self.system_state.UA_mat = IMU_mat
                elif self.segment == "fa":
                    self.system_state.FA_mat = IMU_mat
              
            time.sleep(max(next_time_instant-time.perf_counter(),0))            
            
class computeAssistance(threading.Thread):
    def __init__(self, name, system_state):   
        threading.Thread.__init__(self)
        self.name = name
        self.system_state = system_state
        
    def run(self):
        #Initize Kalman filter for shoulder elvation and elbow flexion
        sh_el_kalman = kalmanFilter()
        el_fe_kalman = kalmanFilter()
        
        counter = 0
        
        while rcpy.get_state() != rcpy.EXITING:
            next_time_instant = time.perf_counter() + DT
            with lock:
                UA_mat_cur = self.system_state.UA_mat
                FA_mat_cur = self.system_state.FA_mat
                        
            sh_el_cur, sh_hr_cur, el_fe_cur = compute_joint_angles(UA_mat_cur,FA_mat_cur)
            sh_el_filt, sh_el_dot = sh_el_kalman.update_kalman(sh_el_cur)
            el_fe_filt, el_fe_dot = el_fe_kalman.update_kalman(el_fe_cur)
            sh_force_des, el_force_des = compute_gravity_assistance(UA_mat_cur,FA_mat_cur,sh_el_filt,el_fe_filt,sh_hr_cur,el_fe_dot)
            
            with lock:
                self.system_state.sh_el = sh_el_cur
                self.system_state.sh_el_dot = sh_el_dot
                self.system_state.el_fe = el_fe_cur
                self.system_state.el_fe_dot = el_fe_dot
                self.system_state.sh_hr = sh_hr_cur   
                self.system_state.sh_force_des = sh_force_des
                self.system_state.el_force_des = el_force_des
     
            counter = counter+1
            if counter%100==0:
                print("Shoulder elevation: ",sh_el_cur*180/np.pi)
                #print("Elbow flexion: ",el_fe_cur*180/np.pi)
                #print("Humeral rotation: ",sh_hr_cur)
                #print("Shoulder force: ", sh_force_des)
                #print("Elbow force: ", el_force_des)
                
            time.sleep(max(next_time_instant-time.perf_counter(),0))

class readBatteryVoltageLoop(threading.Thread):
    def __init__(self, name, system_state):
        threading.Thread.__init__(self)
        self.name = name
        self.system_state = system_state
        self.battery_fs = 1.0 # Only update every second

    def run(self):
        print("Starting Battery monitor thread...")
        while rcpy.get_state() != rcpy.EXITING:
            next_time_instant = time.perf_counter() + (1.0 / self.battery_fs)

            cmd = '/sys/bus/iio/devices/iio:device0/in_voltage6_raw'
            result = subprocess.run(['cat', cmd], stdout=subprocess.PIPE)

            V_fs = 3.3 # Full scale voltage
            N_bits = 4096.0 # 12-bit discretization

            v_in = int(result.stdout[:-1]) * 3.3 / 4096.0

            R1 = 24000.0 # first resistor, Ohms
            R2 = 1300.0 # second resistor, Ohms
            resistor_ratio = 0.05041667 # Calibrated with constant voltage source, better than using resistor values.

            v_bat = v_in / resistor_ratio
            #print("Battery Voltage: {:.2f}".format(v_bat))

            with lock:
                self.system_state.v_bat = v_bat

            if (v_bat < 20.0):
                warnings.warn("Critical Warning: Battery voltage below 20V. Shutting down...")
                rcpy.set_state(rcpy.EXITING)
            elif (v_bat < 21.0):
                warnings.warn("Warning: Battery voltage low (< 21.0 V.). Please charge.")

            time.sleep(max(next_time_instant-time.perf_counter(),0))

class saveDataLoop(threading.Thread):
    def __init__(self, name, sys_state):
        threading.Thread.__init__(self)
        self.name = name
        self.sys_state = sys_state
        self.save_fs = FS

    def run(self):
        t0 = time.perf_counter()

        # Nice to have time-stamped log file if running experiments...
        #F_OUT = open('logs/'+datetime.now().strftime("%Y-%m-%d_%H-%M-%S_") + 'log.csv', 'w')

        # But for debugging / development, have a simple filename to overwrite each time.
        F_OUT = open('logs/log.csv', 'w')
        
        file_header = "t,v_battery," # TDU vitals
        file_header = file_header + "kp_m1,kd_m1,p_des_m1,v_des_m1,t_des_m1," # Motor1 desired state
        file_header = file_header + "p_cur_m1,v_cur_m1,t_cur_m1," # Motor1 actual state
        file_header = file_header + "kp_m2,kd_m2,p_des_m2,v_des_m2,t_des_m2," # Motor2 desired state
        file_header = file_header + "p_cur_m2,v_cur_m2,t_cur_m2," # Motor2 actual state
        file_header = file_header + "sh_el,sh_el_dot," # IMU1
        file_header = file_header + "sh_hr," # IMU2
        file_header = file_header + "el_fe,el_fe_dot," # Elbow flex/ext
        file_header = file_header + "sh_force_des,el_force_des," #Tendon forces
        file_header = file_header + "force_cur," # Load cell
        file_header = file_header + "trigger,gravity_support_shoulder,gravity_support_elbow\n" # trigger state and gravity support level
        F_OUT.write(file_header)

        while rcpy.get_state() != rcpy.EXITING:
            next_time_instant = time.perf_counter() + (1.0 / self.save_fs)
            t = time.perf_counter()-t0

            with lock:
                data = "{:.5f},{:.2f},".format(t, self.sys_state.v_bat)
                data = data + "{:.2f},{:.2f},{:.3f},{:.3f},{:.3f},".format(self.sys_state.control_state[0].kp,self.sys_state.control_state[0].kd,self.sys_state.control_state[0].p_des,self.sys_state.control_state[0].v_des,self.sys_state.control_state[0].t_ff)
                data = data + "{:.3f},{:.3f},{:.3f},".format(self.sys_state.motor_state[0].position,self.sys_state.motor_state[0].velocity,self.sys_state.motor_state[0].torque)
                data = data + "{:.2f},{:.2f},{:.3f},{:.3f},{:.3f},".format(self.sys_state.control_state[1].kp,self.sys_state.control_state[1].kd,self.sys_state.control_state[1].p_des,self.sys_state.control_state[1].v_des,self.sys_state.control_state[1].t_ff)
                data = data + "{:.3f},{:.3f},{:.3f},".format(self.sys_state.motor_state[1].position,self.sys_state.motor_state[1].velocity,self.sys_state.motor_state[1].torque)
                data = data + "{:.3f},{:.3f},".format(self.sys_state.sh_el,self.sys_state.sh_el_dot)
                data = data + "{:.3f},".format(self.sys_state.sh_hr)
                data = data + "{:.3f},{:.3f},".format(self.sys_state.el_fe,self.sys_state.el_fe_dot)
                data = data + "{:.3f},{:.3f},".format(self.sys_state.sh_force_des,self.sys_state.el_force_des)
                data = data + "{:.3f},".format(self.sys_state.force_cur)
                data = data + "{:.3f},{:.2f},{:.2f}\n".format(self.sys_state.trigger,self.sys_state.gravity_support_shoulder,self.sys_state.gravity_support_elbow)
            
            # Write can happen after lock is given up
            F_OUT.write(data)

            time.sleep(max(next_time_instant-time.perf_counter(),0))

        # Close the file once the thread ends
        F_OUT.close()
        
def main():        
    system_state = systemState()
    if SUPPORTED_ARM:
        MOTOR_JOINT_ASSIGNMENT = ["shoulder","elbow"] #RIGHT ARM: motor1 is assigned to shoulder, motor2 is assigned to elbow
        print("Supporting right arm...")
        if SHOULDER_SUPPORT:
            motor1_on = True
        else: 
            motor1_on = False
        if ELBOW_SUPPORT:
            motor2_on = True    
        else:
            motor2_on = False
    else:
        MOTOR_JOINT_ASSIGNMENT = ["elbow","shoulder"] #LEFT ARM: motor1 is assigned to elbow, motor2 is assigned to shoulder
        print("Supporting left arm...")
        if SHOULDER_SUPPORT:
            motor2_on = True
        else: 
            motor2_on = False
        if ELBOW_SUPPORT:
            motor1_on = True    
        else:
            motor1_on = False
    
    # motor1 has CAN ID = 1, and is the left motor.
    motor1 = Motor(CAN_ID = CAN_IDs[0], FS = FS, model = MOTOR_MODELS[0], joint = MOTOR_JOINT_ASSIGNMENT[0])
    motor1.set_control_gains(KP_1, KD_1)
    motor1.set_cutoff_torque(MAX_TORQUE)
    motor1.set_cutoff_speed(MAX_SPEED)
    
    # motor2 has CAN ID = 2, and is the right motor.
    motor2 = Motor(CAN_ID = CAN_IDs[1], FS = FS, model = MOTOR_MODELS[1], joint = MOTOR_JOINT_ASSIGNMENT[1])
    motor2.set_control_gains(KP_2, KD_2)
    motor2.set_cutoff_torque(MAX_TORQUE)
    motor2.set_cutoff_speed(MAX_SPEED)
    
    control_mode = "torque"
    
    # IMU1 is assigned to upper arm, IMU2 to forearm
    imu1 = Imu(IMU_RECEIVE_PORTS[0], IMU_IP_ADDRESSES[0], IMU_SEND_PORT, IMU_AXIS_UP)
    imu2 = Imu(IMU_RECEIVE_PORTS[1], IMU_IP_ADDRESSES[1], IMU_SEND_PORT, IMU_AXIS_UP)
    
    with lock:
        system_state.trigger = 0
    
    print("System initialization...")
    time.sleep(1)

    motor1ControlThread = motorControlLoop("Motor 1 control", system_state, motor1, control_mode)
    motor2ControlThread = motorControlLoop("Motor 2 control", system_state, motor2, control_mode)
    readCANBusLoopThread = readCANBusLoop("CAN Bus Reading", system_state, [motor1,motor2])
    readLoadCellThread = readLoadCellLoop("Read load cell", system_state)
    readImu1Thread = readImuLoop("Read IMU 1", system_state, imu1, "ua")
    readImu2Thread = readImuLoop("Read IMU 2", system_state, imu2, "fa")
    computeAssistanceThread = computeAssistance("Compute assistance", system_state)
    readBatteryVoltageThread = readBatteryVoltageLoop("Read Battery Voltage", system_state)
    saveDataThread = saveDataLoop("Save data", system_state)
    
    threads = []
    if motor1_on:
        threads.append(motor1ControlThread)
    if motor2_on:
        threads.append(motor2ControlThread)
    threads.append(readCANBusLoopThread)
    #threads.append(readLoadCellThread)
    threads.append(readImu1Thread)
    #threads.append(readImu2Thread)
    threads.append(computeAssistanceThread)
    #threads.append(readBatteryVoltageThread)
    threads.append(saveDataThread)
    
    for t in threads:
        t.start()
        
    for t in threads:
        t.join()
            
if __name__ == "__main__":
    try:
        main()
    # If there is any uncaught exception, re-init motors and start/stop them.
    except:
        os.popen('echo 1 > /sys/class/gpio/gpio105/value')
        os.popen('echo 0 > /sys/class/gpio/gpio105/value')
        if SUPPORTED_ARM:
            MOTOR_JOINT_ASSIGNMENT = ["shoulder","elbow"] #RIGHT ARM: motor1 is assigned to shoulder, motor2 is assigned to elbow
        else:
            MOTOR_JOINT_ASSIGNMENT = ["elbow","shoulder"] #LEFT ARM: motor1 is assigned to elbow, motor2 is assigned to shoulder          
        print("Disabling motor!")
        jj=0
        while jj<4:
            motor1 = Motor(CAN_ID = 1, FS = FS, model = "AK60_6", joint = MOTOR_JOINT_ASSIGNMENT[0])
            print('Attempt'+ str(jj+1))
            motor1.enable_and_stop()
            motor1.stop_and_disable()
            
            motor2 = Motor(CAN_ID = 2, FS = FS, model = "AK60_6", joint = MOTOR_JOINT_ASSIGNMENT[1])
            print('Attempt'+ str(jj+1))
            motor2.enable_and_stop()
            motor2.stop_and_disable()
            
            jj+=1
            time.sleep(0.5)
            
        print('M1 and M2 disabled!')

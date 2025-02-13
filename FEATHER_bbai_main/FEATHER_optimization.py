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

import json

f = open('/home/debian/FEATHER/feather/settings.json')
set = json.load(f)
f.close()

# q = open("/home/debian/FEATHER/feather/velocities.csv",'w')

# TODO: Remove before making repo public
# App Password: zebg7samHtRmXLxWRs8U

# Suit geometrical parameters
PULLEY_R = 0.035

# User Data

M = set["physical_settings"][0]["weight"]
H = set["physical_settings"][0]["height"]

# Constants
G = 9.81 # acceleration of gravity
    
# Assistance percentage
ASSIST_PERC = set["preference"][0]["perc_assist"]

# Minimum and Maximum cable tensions (N)
MIN_TENSION = 5.0
MAX_TENSION = 100.0

# Main control loop frequency
FS = 100.0
DT = 1.0 / FS


# Motors specs
CAN_IDs = [1,2]
MODELS = ["AK60_6", "AK80_9"]

# Control parameters
KP_1 = 2.0
KD_1 = 2.5

KP_2 = 20
KD_2 = 3.5#relatively stable at 12 and 2.5 respectively

# Load cell parameters from calibration
LC_OFFSET = 0.074
LC_GAIN = 1/0.00545

# Parameters for friction compensation model
if set["preference"][0]["Friction_compensation"] == True:
    pp = np.loadtxt('/home/debian/FEATHER/feather/parameters.csv', delimiter=',')
else:
    pp = np.loadtxt('/home/debian/FEATHER/feather/par_zero.csv', delimiter=',')

print(pp)

# IMU parameters from NGIMU GUI
IMU_AXIS_UP = 'Y'
IMU_RECEIVE_PORTS = [8001]
IMU_SEND_PORT = 9000
IMU_IP_ADDRESSES = ["192.168.1.1"]

# Thread Lock
lock = threading.Lock()

class systemState():
    v_bat = 0

    # TODO: Put desired state into a struct as well
    kp_m1 = 0
    kd_m1 = 0
    p_des_m1 = 0
    v_des_m1 = 0
    t_ff_m1 = 0

    # TODO: Put desired state into a struct as well
    kp_m2 = 0
    kd_m2 = 0
    p_des_m2 = 0
    v_des_m2 = 0
    t_ff_m2 = 0

    motor1_state = MotorState()
    motor2_state = MotorState()

    acc_m1 = 0
    acc_m2 = 0

    sh_el = 0
    sh_el_dot = 0
    #sh_el_dot_back = 0
    #sh_el_dot_sgol = 0
    #sh_el_dot_dsp1 = 0
    #sh_el_dot_dsp2 = 0

    force_cur = 0
    force_des = 0

def filter(x):
    """Filter incoming data with cascaded second-order sections.
    """
    sos=scipy.signal.butter(1, 1.5, btype='lowpass', analog=False, fs=100, output='sos') # filter @100Hz
    n_sections = sos.shape[0]
    state = np.zeros((n_sections, 2))

    for s in range(n_sections):  # apply filter sections in sequence
        b0, b1, b2, a0, a1, a2 = sos[s, :]
        # compute difference equations of transposed direct form II
        y = b0*x + state[s, 0]
        state[s, 0] = b1*x - a1*y + state[s, 1]
        state[s, 1] = b2*x - a2*y
        x = y  # set biquad output as input of next filter section.

    return y

class ExpFilter():
    def __init__(self, tau):
        self.tau = tau
        self._xs = 0
        self._ys = 0
        
    def _process(self, x):
        #Filter incoming data with exponential filter
        self._xs = x
        y = y = self.tau*x + (1-self.tau)*self._ys
        self._ys = y
        return y
    
    
def compute_gravity_assistance(theta):

    # Cap angle at 120 degrees to avoid modelling singularity at ~170 ish degrees
    theta = np.minimum(theta, 120.0*np.pi/180.0)

    # Anthropometry
    h = 0.06 # Height of cable exit above skin
    r_ac = 0.0430 * H # Radius of rotation of GH joint (GH to acromion)
    m_arm = 0.050 * M
    l_arm = 0.3320 * H
    l_com_arm = 0.530 * l_arm
    upper_arm_length = 0.186 * H
    l_anchor = 0.7 * upper_arm_length # Distance to anchor point along humerus
    tau_gravity_max_arm = m_arm * l_com_arm * G
    w = np.sqrt(l_anchor**2 + (0.1*r_ac)**2)
    phi = np.arctan2(r_ac, l_anchor)
    num = w - (h + r_ac) * np.cos(np.pi - theta - phi)
    den = np.sqrt( (h + r_ac)**2 + w**2 - 2*(h + r_ac) * w * np.cos(np.pi - theta - phi) )
    alpha = np.arccos(num / den)
    tau_req = tau_gravity_max_arm * np.sin(theta)
    
    # Desired tendon tension
    T_tendon = tau_req / (w * np.sin(alpha)) * ASSIST_PERC

    # Sigmoid to smoothen gravity model at lower angles
    '''
    v_1_pct = 0.0
    v_99_pct = 40.0*np.pi/180.0
    shift = (v_1_pct + v_99_pct) / 2
    rise = -np.log(99) / (v_1_pct - shift)
    sigmoid_pct = 1.0 / (1 + np.exp(-rise*(theta - shift)))
    '''

    # Less support at low angles, and smoothly ramp to full gravity function.
    #T_tendon = sigmoid_pct * T_tendon
    T_tendon = T_tendon

    # Clip output tension (pretension and safety)
    T_tendon = np.clip(T_tendon, MIN_TENSION, MAX_TENSION)

    return T_tendon
    
def compute_friction_compensation(T_tendon_des, sh_el_cur, sh_el_dot_cur, v_cur_m1):
    ## New Model
    r_pulley=0.035
    fc = pp[0]
    fs = pp[1]
    v_brk = pp[2]
    delta = pp[3]
    alpha = pp[4]
    
    sh_tdu_vel_ratio = 2.3199
    #sh_vel=filter(sh_el_dot_cur)
    sh_vel = sh_el_dot_cur
    
    v_cable = sh_vel * sh_tdu_vel_ratio * r_pulley
    
    f = (fc + (fs-fc)*np.exp(-np.abs(v_cable/v_brk)**delta))*np.sign(v_cable)
    
    T_in = ( ( T_tendon_des - T_tendon_des*f + 2*T_tendon_des*f*( 2 / (1 + np.exp(-np.abs(10000 * v_cable))) - 1 ) ) / (1-f) ) / (1-alpha)
    #T_in = T_tendon_des*(1+f)/(1-f)/(1-alpha)
    tau_des = T_in*r_pulley
    
    
    
    
    # # Desired tendon force to motor torque models
    # #m_average = 0.0333 # slope, Nm / m, pulley 0.035
    # #m_lowering = 0.0141 # slope, Nm / m
    # #m_raising = 0.0524 # slope, Nm / m

    # # From Elena (2022-12-02 fit)
    # m_raising = 0.0343
    # m_lowering = 0.0219

    # b_raising = 0.0792
    # b_lowering = -0.0620

    # # One single sigmoid with negative sat when at rest.
    # v_001 = 0.0
    # v_099 = 4.0
    # #v_001 = -4.0
    # #v_099 = 0.0

    # M1 = (v_001 + v_099) / 2
    # B1 = -np.log(99) / (v_001 - M1)    
    # sh_el_dot_cur = np.clip(sh_el_dot_cur, -5, 5) # Just to stop overflow...
 
    # sigmoid_lower = b_lowering + m_lowering*T_tendon_des
    # sigmoid_upper = b_raising + m_raising*T_tendon_des

    # # With friction compensation
    # #tau_pct = m_lowering + (m_raising-m_lowering) / (1 + np.exp(-B1*(sh_el_dot_cur - M1)))
    # #tau_des = tau_pct * T_tendon_des
    # tau_des = sigmoid_lower + (sigmoid_upper-sigmoid_lower) / (1 + np.exp(-B1*(sh_el_dot_cur - M1)))

    # # Without friction compensation
    # #tau_des = PULLEY_R * T_tendon_des
    
    
    # pp = [0, 0, 0, 0, 0, 0, 0]
    # pp = [0.006556712417582489, 2500.0, 377.6935692083482, 4.66778617956065, 441.94500432252755, 0.48655422998080994, 0.1056057190316948]
    #vel=filter(sh_el_dot_cur)
    # vel=sh_el_dot_cur
    #pos=sh_el_cur
    #pv=[4.937061969686603, -10.196369713358601, 1.648692762607444]
    # pv=[-1.280463968738144, 0.028897120527366]
    # rope_vel=r_pulley*pv[0]*vel**2+pv[1]*vel
    #rope_vel=-r_pulley*(pv[2]*vel+pv[1]*pos*vel+pv[0]*vel*pos**2) # minus beacuse the optimization is performed on the velocity of the motor which is positive when unwinding/descending
    #q.write(str(vel) + ',' + str(rope_vel/r_pulley) + '\n')
    # rope_vel=filter(-1.0*v_cur_m1) # why without r_pulley?
    #cfh = (pp[0] + (pp[1]-pp[0]) * np.exp(-(np.abs(rope_vel/pp[2]))**pp[3] ))*np.sign(rope_vel)
    #Tfr = (2*T_tendon_des*cfh)/(1-cfh)
    #Tfriction = Tfr * ( 2 / (1 + np.exp(-np.abs(75000 * rope_vel))) - 1 )
    #cf = (pp[0]*(np.tanh(pp[1]*rope_vel)-np.tanh(pp[2]*rope_vel))+pp[3]*np.tanh(pp[4]*rope_vel))
    #Tfriction = (2*T_tendon_des*pp[5]*cf)/(1-pp[5]*cf)
    #tau_des=(T_tendon_des+Tfriction)*r_pulley
    
    
    tau_des=np.clip(tau_des, 0.1, 9.0)

    return tau_des

def compute_inertial_compensation(tau_des, motor_acc):
    inertia_comp = 1.0*motor_acc*0.0017 # + 0.0564*np.sign(motor_vel)
    return tau_des + inertia_comp


class motorControlLoop(threading.Thread):
    def __init__(self, name, sys_state):
        threading.Thread.__init__(self)
        self.name = name
        self.sys_state = sys_state

    # NOTE: For the integration of the second motor, I will for now
    #       integrate it into the motor control loop thread. 
    #       This is not super ideal, since it complicates the thread 
    #       quite a bit, but avoids needing a higher software framework
    #       to manage both motors (maybe a TODO for the future). 
    #       Right now the integration will be quite specific for the study
    #       protocol, where the second motor will drive the mannequin arm
    #       in trajectory control, and the first motor will provide 
    #       gravity support.
    def run(self):
        # define filters
        exp_filter_sh_el_dot = ExpFilter(0.2) #exponential filter
    
        # motor1 has CAN ID = 1, and is the TDU motor.
        motor1_model = "AK60_6"
        motor1 = Motor(CAN_ID = 1, FS = FS, model = motor1_model)
        motor1.set_control_gains(KP_1, KD_1)
        motor1.set_cutoff_torque(9.0)
        motor1.set_cutoff_speed(20)
        motor1.stop_and_disable()
        time.sleep(0.1)

        # NOTE: Taring is not done in the motor object since the position 
        #       is updated in the main system state. This is preferable 
        #       to updating the state in the motor object, because then
        #       the lock would be required whenever accessing the motor objects.
        #       Also this way needs duplicated code :(
        with lock:
            motor1_cur_pos = self.sys_state.motor1_state.position
        print("Taring Motor1...")
        print("Motor1 Current Position: {}".format(motor1_cur_pos))        
        while (np.abs(motor1_cur_pos) > 0.1):
            motor1.stop_and_disable()
            motor1.tare_position()
            with lock:
                motor1_cur_pos = self.sys_state.motor1_state.position
        print("Motor1 Tared Position: {}".format(motor1_cur_pos))        

        # motor2 has CAN ID = 2, and is the external motor.
        motor2_model = "AK80_9"
        motor2 = Motor(CAN_ID = 2, FS = FS, model = motor2_model)
        motor2.set_control_gains(KP_2, KD_2)
        motor2.set_cutoff_torque(12.0)
        motor2.set_cutoff_speed(20)
        motor2.stop_and_disable()
        time.sleep(0.1)

        # And tare the second motor in the same way
        with lock:
            motor2_cur_pos = self.sys_state.motor2_state.position
        print("Taring Motor2...")
        print("Motor2 Current Position: {}".format(motor2_cur_pos))        
        while (np.abs(motor2_cur_pos) > 0.1):
            motor2.stop_and_disable()
            motor2.tare_position()
            with lock:
                motor2_cur_pos = self.sys_state.motor2_state.position
        print("Motor2 Tared Position: {}".format(motor2_cur_pos))        

        # TODO: Probably stupid to have both running in trajectory mode...
        #       But if this is necessary, then ensure that they share the file
        #       appropriately (and safely...)
        control_mode_motor1 = "torque"
        control_mode_motor2 = "trajectory"

        # Counter for trajectory mode (to repeat if wished)
        row_counter = 0
        cycle_counter = 0
        n_cycles = 1

        # Save the chosen control gains in the system state
        # TODO: Clean up and do this nicer.
        self.sys_state.kp_m1 = KP_1
        self.sys_state.kd_m1 = KD_1
        self.sys_state.kp_m2 = KP_2
        self.sys_state.kd_m2 = KD_2

        # Import csv file that contains reference trajectory
        if (control_mode_motor1 == "trajectory" or control_mode_motor2 == "trajectory"):
            time.sleep(5)

            #print("Are you performing with full range of forces - up to 50 N (y/n)? ")
            #resp = input()

            #while resp != 'y' and resp != 'Y' and resp != 'n' and resp != 'N' and resp != '':
            #    print('Not a valid input, retry: is this the movement with full ROM (y/n)?')
            #    resp=input()
            
            #if resp == 'y' or resp == 'Y' or resp == '':
            #    trajfile = "/home/debian/FEATHER/feather/mannequin_trajectories/fullROM_40.csv"
            #else:
            #    trajfile = "/home/debian/FEATHER/feather/mannequin_trajectories/fullROM_30.csv"
            trajfile = "/home/debian/FEATHER/feather/mannequin_trajectories/fullROM_40.csv"
            ref_traj = np.loadtxt(open(trajfile, "r"), delimiter=',')
            n_rows = np.shape(ref_traj)[0]

        # Pretension
        is_system_pretensioned = False
        motor1.set_control_mode("velocity", self.sys_state.kp_m1, self.sys_state.kd_m1)
        #cable_pretension_setpoint = compute_gravity_assistance(5.0*np.pi/180.0)
        cable_pretension_setpoint = 5 # Hardcode tension for now
        print("Pretension Setpoint: ", cable_pretension_setpoint, " N.")
        # Desired State
        p_des = 0.0
        v_des = 0.0
        t_ff = 0.0
        force_des = 0.0

        # Setup for motor2 before entering main loop
        motor2.set_control_mode("trajectory", self.sys_state.kp_m2, self.sys_state.kd_m2)

        # Now finally enable both motors
        motor1.enable_and_stop()
        motor2.enable_and_stop()

        # MAIN CONTROL LOOP
        while rcpy.get_state() != rcpy.EXITING:
            next_time_instant = time.perf_counter() + DT

            # At the beginning of every loop, we update the motor objects
            # with the most current state from sys_state.
            # NOTE: Maybe we don't need to do this, since currently the motor object never
            #       needs its internal state (only to update the filtered torque, where the 
            #       value is anyway passed in...). So this could be removed.
            with lock:
                motor1.motor_state.can_id = self.sys_state.motor1_state.can_id
                motor1.motor_state.position = self.sys_state.motor1_state.position
                motor1.motor_state.velocity = self.sys_state.motor1_state.velocity
                motor1.motor_state.torque = self.sys_state.motor1_state.torque
                motor1.motor_state.temperature = self.sys_state.motor1_state.temperature
                motor1.motor_state.error_code = self.sys_state.motor1_state.error_code

                motor2.motor_state.can_id = self.sys_state.motor2_state.can_id
                motor2.motor_state.position = self.sys_state.motor2_state.position
                motor2.motor_state.velocity = self.sys_state.motor2_state.velocity
                motor2.motor_state.torque = self.sys_state.motor2_state.torque
                motor2.motor_state.temperature = self.sys_state.motor2_state.temperature
                motor2.motor_state.error_code = self.sys_state.motor2_state.error_code

                force_cur=self.sys_state.force_cur

            # Torque Safety Cutoffs
            t1_cur_filt = motor1.update_filtered_torque(motor1.motor_state.torque)  
            t2_cur_filt = motor2.update_filtered_torque(motor2.motor_state.torque)

            # TODO: Do nicer...                        
            a1_acc_filt = motor1.update_filtered_acc(motor1.motor_state.velocity)
            a2_acc_filt = motor2.update_filtered_acc(motor2.motor_state.velocity)

            with lock:
                self.sys_state.acc_m1 = a1_acc_filt
                self.sys_state.acc_m2 = a2_acc_filt

            torque_cutoff_condition_m1 = np.abs(t1_cur_filt) > motor1.safety_limits.cutoff_torque
            torque_cutoff_condition_m2 = np.abs(t2_cur_filt) > motor2.safety_limits.cutoff_torque
            if (torque_cutoff_condition_m1 or torque_cutoff_condition_m2):
                warnings.warn("Cutoff torque exceeded, system exiting...")
                if (torque_cutoff_condition_m1):
                    print('Over-the-limit on motor 1')
                else:
                    print('Over-the-limit on motor 2')
                rcpy.set_state(rcpy.EXITING)

            # Speed Safety Cutoffs
            speed_cutoff_condition = np.abs(motor1.motor_state.velocity) > motor1.safety_limits.cutoff_speed or np.abs(motor2.motor_state.velocity) > motor2.safety_limits.cutoff_speed
            if (speed_cutoff_condition):
                warnings.warn("Cutoff velocity exceeded, system exiting...")
                rcpy.set_state(rcpy.EXITING)

            '''
            # Temperature Safety Cutoffs
            temperature_cutoff_condition = motor1.motor_state.temperature > motor1.safety_limits.cutoff_temperature or motor2.motor_state.temperature > motor2.safety_limits.cutoff_temperature
            if (temperature_cutoff_condition):
                warnings.warn("Cutoff temperature exceeded, system exiting...")
                rcpy.set_state(rcpy.EXITING)
            '''

            '''
            # Position Safety Cutoffs Motor1 (TDU)
            position_cutoff_condition_m1 = motor1.motor_state.position > 8.0 or motor1.motor_state.position < -0.5
            if (position_cutoff_condition_m1):
                warnings.warn("Cutoff position exceeded (M1), system exiting...")
                rcpy.set_state(rcpy.EXITING)

            # Position Safety Cutoffs Motor2 (TDU)
            position_cutoff_condition_m2 = motor2.motor_state.position > 0.1 or motor2.motor_state.position < -2.0
            if (position_cutoff_condition_m2):
                warnings.warn("Cutoff position exceeded (M2), system exiting...")
                rcpy.set_state(rcpy.EXITING)
            '''

            p_des_m1 = 0
            v_des_m1 = 0
            t_ff_m1 = 0

            p_des_m2 = 0
            v_des_m2 = 0
            t_ff_m2 = 0

            # PRETENSION BLOCK (only executes once)
            if is_system_pretensioned == False:
                # motor1
                v_des_m1 = -0.5
                #t_cable = np.abs(t1_cur_filt) / PULLEY_R 
                t_cable=force_cur
                t_ff_m1 = 0
                motor1.set_ref_velocity(v_des_m1, t_ff_m1)
                print("t_cable: ", t_cable)

                # motor2 (blocked until pretension done)
                motor2.set_ref_trajectory(p_des_m2, v_des_m2, t_ff_m2)

                if t_cable > cable_pretension_setpoint:
                    is_system_pretensioned = True
                    # Set motor control type: "trajectory", "torque", "velocity"
                    motor1.set_control_mode(control_mode_motor1, self.sys_state.kp_m1, self.sys_state.kd_m1)
                    motor2.set_control_mode(control_mode_motor2, self.sys_state.kp_m2, self.sys_state.kd_m2)

            else: # MAIN CONTROL LOOP AFTER PRETENSION ACHIEVED
                
                # motor2
                p_des_m2 = 0.0
                v_des_m2 = 0.0
                t_ff_m2 = 0.0
                if control_mode_motor2 == "trajectory":
                    cur_row = ref_traj[row_counter]
                    p_des_m2 = cur_row[0]
                    v_des_m2 = cur_row[1]
                    t_ff_m2 = 0.0
                    motor2.set_ref_trajectory(p_des_m2, v_des_m2, t_ff_m2)
                    row_counter = row_counter + 1  
                    if (row_counter == n_rows):
                        row_counter = 0 # Re-start reading trajectory
                        cycle_counter = cycle_counter + 1

                        if (cycle_counter >= n_cycles):
                            rcpy.set_state(rcpy.EXITING)

                elif control_mode_motor2 == "velocity":
                    with lock:
                        sh_el_cur = self.sys_state.sh_el
                    
                    v_des_m2 = 10.0 * sh_el_cur / (np.pi)
                    t_ff_m2 = 0.0
                    motor2.set_ref_velocity(v_des_m2, t_ff_m2)

                elif control_mode_motor2 == "torque":
                    with lock:
                        sh_el_cur = self.sys_state.sh_el
                    
                    t_ff_m2 = 0.0
                    motor2.set_ff_torque(t_ff_m2)
                
                # motor1
                if control_mode_motor1 == "trajectory":
                    cur_row = ref_traj[row_counter]
                    p_des_m1 = cur_row[0]
                    v_des_m1 = cur_row[1]
                    t_ff_m1 = cur_row[2]
                    motor1.set_ref_trajectory(p_des_m1, v_des_m1, t_ff_m1)
                    row_counter = row_counter + 1  
                    if (row_counter == n_rows):
                        row_counter = 0 # Re-start reading trajectory
                        cycle_counter = cycle_counter + 1

                        if (cycle_counter >= n_cycles):
                            rcpy.set_state(rcpy.EXITING)

                elif control_mode_motor1 == "velocity":
                    with lock:
                        sh_el_cur = self.sys_state.sh_el
                    
                    v_des_m1 = 20.0 * sh_el_cur / np.pi
                    t_ff_m1 = 0.0
                    v_des_m1 = 0.0
                    motor1.set_ref_velocity(v_des_m1, t_ff_m1)

                elif control_mode_motor1 == "torque":
                    p_des_m1 = 0
                    v_des_m1 = 0
                    
                    with lock:
                        sh_el_cur = self.sys_state.sh_el
                        sh_el_dot_cur = self.sys_state.sh_el_dot 
                        force_cur = self.sys_state.force_cur
                        v_cur_m1 = self.sys_state.motor1_state.velocity

                    #force_des = compute_gravity_assistance(sh_el_cur)
                    #force_des = 50
                    force_des = cur_row[2]
                    sh_el_dot_cur_filt = exp_filter_sh_el_dot._process(sh_el_dot_cur)
                    
                    #force_des = 5
                    r_pulley = 0.035
                    t_ff_m1 = force_des * r_pulley
                    
                    # Adding the motor model
                    #if np.abs(v_cur_m1) <= 0.015:
                    #    t_ff_m1 = t_ff_m1 + 0.2 * np.sign(v_cur_m1)
                    #else:
                    #    t_ff_m1 = t_ff_m1 + 0.07 * np.sign(v_cur_m1)

                    if set["preference"][0]["Friction_compensation"] == True:
                        t_ff_m1 = compute_friction_compensation(force_des, sh_el_cur, sh_el_dot_cur_filt, v_cur_m1)
                    #t_ff_m1 = compute_inertial_compensation(t_ff_m1, a1_acc_filt)

                    # Reverse direction of output on new box
                    t_ff_m1 = -1.0* t_ff_m1
                    #t_ff_m1 = -0.5

                    motor1.set_ff_torque(t_ff_m1)

            # Save desired state to system
            with lock:
                # motor1 desired state (for logging)
                self.sys_state.p_des_m1 = p_des_m1
                self.sys_state.v_des_m1 = v_des_m1
                self.sys_state.t_ff_m1 = t_ff_m1

                # motor2 desired state (for logging)
                self.sys_state.p_des_m2 = p_des_m2
                self.sys_state.v_des_m2 = v_des_m2
                self.sys_state.t_ff_m2 = t_ff_m2

                self.sys_state.force_des = force_des

            time.sleep(max(next_time_instant-time.perf_counter(),0))

        print("Disabling motor!")
        ii=0
        while ii<4:
            print('Attempt'+ str(ii+1))
            motor1.stop_and_disable()
            ii+=1
            time.sleep(0.5)
        print('M1 disabled!')        
        motor2.stop_and_disable()
        print('M2 disabled!')
            
class readCANBusLoop(threading.Thread):
    def __init__(self, name, sys_state):
        threading.Thread.__init__(self)
        self.name = name
        self.sys_state = sys_state
        self.can_read_fs = 2 * FS # Run thread faster than main controller so that messages are always up-to-date.

        # The motors do not respond with the CAN ID in the correct arbitration field
        # but rather in the first byte of data. And the arbitration field is 0x000
        # for motor responses, so we filter for these messages (so we don't pick up
        # sent commands on the CAN bus)
        filters = [{"can_id": 0x000, "can_mask": 0x7FF, "extended": False}]
        self.bus = Bus(channel="can0", bustype="socketcan", can_filters=filters, bitrate=1000000)

        # Temporary motor state placeholders
        self.motor1_state_cur = MotorState()
        self.motor2_state_cur = MotorState()

    def run(self):
        while rcpy.get_state() != rcpy.EXITING:
            next_time_instant = time.perf_counter() + (1.0 / self.can_read_fs) 

            cur_msg = self.bus.recv(timeout = 0) # TODO: What value for timeout?
        
            while cur_msg is not None:
                #print(binascii.hexlify(cur_msg.data))
                can_id, position, velocity, torque, temperature, error_code = tcan.unpack_reply(cur_msg.data, CAN_IDs, MODELS)
                # NOTE: Not ideal to hard-code CAN IDs... But tricky to fix since the motor objects hold the CAN IDs...
                if (can_id == 1):
                    self.motor1_state_cur.can_id = can_id
                    self.motor1_state_cur.position = position
                    self.motor1_state_cur.velocity = velocity
                    self.motor1_state_cur.torque = torque
                    self.motor1_state_cur.temperature = temperature
                    self.motor1_state_cur.error_code = error_code
                elif (can_id == 2):
                    self.motor2_state_cur.can_id = can_id
                    self.motor2_state_cur.position = position
                    self.motor2_state_cur.velocity = velocity
                    self.motor2_state_cur.torque = torque
                    self.motor2_state_cur.temperature = temperature
                    self.motor2_state_cur.error_code = error_code

                # Attempt to get the next message
                cur_msg = self.bus.recv(timeout = 0)
            
            with lock:
                # Update first motor
                self.sys_state.motor1_state.can_id = self.motor1_state_cur.can_id
                self.sys_state.motor1_state.position = self.motor1_state_cur.position
                self.sys_state.motor1_state.velocity = self.motor1_state_cur.velocity
                self.sys_state.motor1_state.torque = self.motor1_state_cur.torque
                self.sys_state.motor1_state.temperature = self.motor1_state_cur.temperature
                self.sys_state.motor1_state.error_code = self.motor1_state_cur.error_code

                # And then the second motor
                self.sys_state.motor2_state.can_id = self.motor2_state_cur.can_id
                self.sys_state.motor2_state.position = self.motor2_state_cur.position
                self.sys_state.motor2_state.velocity = self.motor2_state_cur.velocity
                self.sys_state.motor2_state.torque = self.motor2_state_cur.torque
                self.sys_state.motor2_state.temperature = self.motor2_state_cur.temperature
                self.sys_state.motor2_state.error_code = self.motor2_state_cur.error_code

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
    def __init__(self, name, system_state):
        threading.Thread.__init__(self)
        self.name = name
        self.system_state = system_state
        self.imu_fs = 200 # IMU updates at 100Hz, no point in reading any faster
        self.thorax_id = 0
        self.upperarm_id = 0
        self.forearm_id = 1

    def run(self):
        print("Starting IMU reading thread...")
        imus = []
        for index in range(len(IMU_IP_ADDRESSES)):
            imu = Imu(IMU_RECEIVE_PORTS[index], IMU_IP_ADDRESSES[index], IMU_SEND_PORT, IMU_AXIS_UP)
            dt = 1.0 / self.imu_fs
            imu.initialize(dt)
            imu.identify() # Strobe IMU leds to identify it
            imus.append(imu)
            #time.sleep(1)
        
        while rcpy.get_state() != rcpy.EXITING:
            next_time_instant = time.perf_counter() + (1.0 / self.imu_fs)

            # Get the IMUs rotation matrices
            #TH = imus[self.thorax_id].get_rot_matrix()
            #UA = imus[self.upperarm_id].get_rot_matrix()
            #FA = imus[self.forearm_id].get_rot_matrix()

            # Get the current shoulder elevation angle
            sh_el, sh_el_dot_gyro = imus[self.upperarm_id].read_sh_elev_angle()
            #print("Shoulder elevation angle: ", sh_el) 

            # Compute the time derivative (best method so far)
            sh_el_dot_back = imus[self.upperarm_id].compute_sh_elev_dot_back(sh_el)

            # These methods aren't as good.
            #sh_el_dot_sgol = imu.compute_sh_elev_dot_sgol(sh_el)
            #sh_el_dot_dsp1 = imu.compute_sh_el_dot_dsp1(sh_el)
            #sh_el_dot_dsp2 = imu.compute_sh_el_dot_dsp2(sh_el)

            with lock:
                self.system_state.sh_el = sh_el
                #self.system_state.sh_el_dot = sh_el_dot_back
                self.system_state.sh_el_dot = sh_el_dot_gyro
                #self.system_state.sh_el_dot_sgol = sh_el_dot_sgol
                #self.system_state.sh_el_dot_dsp1 = sh_el_dot_dsp1
                #self.system_state.sh_el_dot_dsp2 = sh_el_dot_dsp2

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
        F_OUT = open('/home/debian/FEATHER/feather/logs/log.csv', 'w')

        file_header = "t,v_battery," # TDU vitals
        file_header = file_header + "kp_m1,kd_m1,p_des_m1,v_des_m1,t_des_m1," # Motor1 desired state
        file_header = file_header + "p_cur_m1,v_cur_m1,t_cur_m1," # Motor1 actual state
        file_header = file_header + "a_m1," # Motor1 acceleration
        file_header = file_header + "kp_m2,kd_m2,p_des_m2,v_des_m2,t_des_m2," # Motor2 desired state
        file_header = file_header + "p_cur_m2,v_cur_m2,t_cur_m2," # Motor2 actual state
        file_header = file_header + "a_m2," # Motor1 acceleration
        file_header = file_header + "sh_el,sh_el_dot," # IMU
        file_header = file_header + "force_des,force_cur\n" # Load cell
        F_OUT.write(file_header)

        while rcpy.get_state() != rcpy.EXITING:
            next_time_instant = time.perf_counter() + (1.0 / self.save_fs)
            t = time.perf_counter()-t0

            with lock:
                data = "{:.5f},{:.2f},".format(t, self.sys_state.v_bat)
                data = data + "{:.2f},{:.2f},{:.3f},{:.3f},{:.3f},".format(self.sys_state.kp_m1,self.sys_state.kd_m1,self.sys_state.p_des_m1,self.sys_state.v_des_m1,self.sys_state.t_ff_m1)
                data = data + "{:.3f},{:.3f},{:.3f},".format(self.sys_state.motor1_state.position,self.sys_state.motor1_state.velocity,self.sys_state.motor1_state.torque)
                data = data + "{:.3f},".format(self.sys_state.acc_m1)
                data = data + "{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},".format(self.sys_state.kp_m2,self.sys_state.kd_m2,self.sys_state.p_des_m2,self.sys_state.v_des_m2,self.sys_state.t_ff_m2)
                data = data + "{:.3f},{:.3f},{:.3f},".format(self.sys_state.motor2_state.position,self.sys_state.motor2_state.velocity,self.sys_state.motor2_state.torque)
                data = data + "{:.3f},".format(self.sys_state.acc_m2)
                data = data + "{:.3f},{:.3f},".format(self.sys_state.sh_el,self.sys_state.sh_el_dot)
                data = data + "{:.3f},{:.3f}\n".format(self.sys_state.force_des,self.sys_state.force_cur)
            
            # Write can happen after lock is given up
            F_OUT.write(data)

            time.sleep(max(next_time_instant-time.perf_counter(),0))

        # Close the file once the thread ends
        F_OUT.close()
        
def main():        
    system_state = systemState()
    print("System initialization...")
    time.sleep(1)

    motorControlThread = motorControlLoop("Motor control", system_state)
    readCANBusLoopThread = readCANBusLoop("CAN Bus Reading", system_state)
    readLoadCellThread = readLoadCellLoop("Read load cell", system_state)
    readImuThread = readImuLoop("Read IMU", system_state)
    readBatteryVoltageThread = readBatteryVoltageLoop("Read Battery Voltage", system_state)
    saveDataThread = saveDataLoop("Save data", system_state)
    
    threads = []
    threads.append(motorControlThread)
    threads.append(readCANBusLoopThread)
    threads.append(readLoadCellThread)
    threads.append(readImuThread)
    # threads.append(readBatteryVoltageThread)
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
        
        print("Disabling motor!")
        jj=0
        while jj<4:
            motor1 = Motor(CAN_ID = 1, FS = FS, model = "AK60_6")
            print('Attempt'+ str(jj+1))
            motor1.enable_and_stop()
            motor1.stop_and_disable()
            jj+=1
            time.sleep(0.5)
        print('M1 disabled!')        
        motor2 = Motor(CAN_ID = 2, FS = FS, model = "AK80_9")
        motor2.enable_and_stop()
        motor2.stop_and_disable()
        print('M2 disabled!')

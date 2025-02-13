#!/usr/bin/python3

from tdu import Motor
from tdu import MotorState
from tdu import Imu
from tdu import loadCell

from pythonosc import udp_client

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

q = open("velocities.csv",'w')

import json

f = open('/home/debian/FEATHER/feather/settings.json')
set = json.load(f)
f.close()

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
CAN_IDs = [1]
MODELS = ["AK60_6"]

# Control parameters for position control
KP_1 = 2.0
KD_1 = 2.5

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

    acc_m1 = 0

    sh_el = 0
    sh_el_dot = 0

    force_cur = 0
    force_des = 0
    
    trigger = 0
    
    gravity_support_level = ASSIST_PERC

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

    # Clip output tension (pretension and safety)
    T_tendon = np.clip(T_tendon, MIN_TENSION, MAX_TENSION)

    return T_tendon

def compute_friction_compensation_simple(T_tendon_des, sh_el_cur, sh_el_dot_cur, v_cur_m1):
    
    ## Desired tendon force to motor torque models
    m_average = 0.0333 # slope, Nm / m, pulley 0.035
    m_lowering = 0.0141 # slope, Nm / m
    m_raising = 0.0524 # slope, Nm / m


    ## One single sigmoid with negative sat when at rest.
    v_001 = 0.0
    v_099 = 4.0

    M1 = (v_001 + v_099) / 2
    B1 = -np.log(99) / (v_001 - M1)    
    sh_el_dot_cur = np.clip(sh_el_dot_cur, -5, 5) # Just to stop overflow...
 
    sigmoid_lower = b_lowering + m_lowering*T_tendon_des
    sigmoid_upper = b_raising + m_raising*T_tendon_des

    ## With friction compensation
    tau_pct = m_lowering + (m_raising-m_lowering) / (1 + np.exp(-B1*(sh_el_dot_cur - M1)))
    tau_des = tau_pct * T_tendon_des
    # tau_des = sigmoid_lower + (sigmoid_upper-sigmoid_lower) / (1 + np.exp(-B1*(sh_el_dot_cur - M1)))

    ## Without friction compensation
    # tau_des = PULLEY_R * T_tendon_des
    
    tau_des=np.clip(tau_des, MIN_TORQUE, MAX_TORQUE)

    return tau_des
    
def compute_friction_compensation(T_tendon_des, sh_el_cur, sh_el_dot_cur, v_cur_m1):  
    
    r_pulley=PULLEY_R

    alpha = pp[4]
    fs = pp[1]
    fc = pp[0]
    v_brk = pp[2]
    delta = pp[3]
    
    # TO DO: make these automatically updated from parameters file
    sh_tdu_vel_ratio = 1.3147 # this actually depends on the anthropometry of the user --> possibly make it data-driven?
    v_thr = 0.02
    
    #sh_vel=filter(sh_el_dot_cur)
    sh_vel = sh_el_dot_cur
    v_cable = sh_vel * sh_tdu_vel_ratio * r_pulley
    
    # Modified controller to take into account TDU-IMU speed shift. Does not need sigmoid
    if (np.abs(v_cable) <= v_thr):
        f = fs*(v_cable/v_thr)
    else:
        f = (fc + (fs-fc)*np.exp(-np.abs((v_cable-np.sign(v_cable)*v_thr)/v_brk)**delta))*np.sign(v_cable)
    
    T_in = T_tendon_des * (1+f)/(1-f)/(1-alpha)
    
    tau_des = T_in*r_pulley
    tau_des=np.clip(tau_des, MIN_TORQUE, MAX_TORQUE)

    return tau_des

def compute_inertial_compensation(tau_des, motor_acc):
    inertia_comp = 1.0*motor_acc*0.0017 + 0.0564*np.sign(motor_vel)
    return tau_des + inertia_comp


class motorControlLoop(threading.Thread):
    def __init__(self, name, sys_state):
        threading.Thread.__init__(self)
        self.name = name
        self.sys_state = sys_state

    def run(self):
        # motor1 has CAN ID = 1, and is the TDU motor.
        motor1_model = "AK60_6"
        motor1 = Motor(CAN_ID = 1, FS = FS, model = motor1_model)
        motor1.set_control_gains(KP_1, KD_1)
        motor1.set_cutoff_torque(MAX_TORQUE)
        motor1.set_cutoff_speed(MAX_SPEED)
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

        control_mode_motor1 = "torque"

        # Counter for trajectory mode (to repeat if wished)
        row_counter = 0
        cycle_counter = 0
        n_cycles = 1

        # Save the chosen control gains in the system state
        # TODO: Clean up and do this nicer.
        self.sys_state.kp_m1 = KP_1
        self.sys_state.kd_m1 = KD_1
        
        # Define Pretension Details
        is_system_pretensioned = False
        motor1.set_control_mode("velocity", self.sys_state.kp_m1, self.sys_state.kd_m1)
        
        cable_pretension_setpoint = PRETENSION
        print("Pretension Setpoint: ", cable_pretension_setpoint, " N.")
        
        # Init Desired State
        p_des = 0.0
        v_des = 0.0
        t_ff = 0.0
        force_des = 0.0

        # Now finally enable motor
        motor1.enable_and_stop()
        
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

                force_cur=self.sys_state.force_cur

            # Torque Safety Cutoffs
            t1_cur_filt = motor1.update_filtered_torque(motor1.motor_state.torque)

            # TODO: Do nicer...                        
            a1_acc_filt = motor1.update_filtered_acc(motor1.motor_state.velocity)

            with lock:
                self.sys_state.acc_m1 = a1_acc_filt

            torque_cutoff_condition_m1 = np.abs(t1_cur_filt) > motor1.safety_limits.cutoff_torque
            
            if (torque_cutoff_condition_m1):
                warnings.warn("Cutoff torque exceeded, system exiting...")
                print('Over-the-limit on motor 1')
                rcpy.set_state(rcpy.EXITING)

            # Speed Safety Cutoffs
            speed_cutoff_condition = np.abs(motor1.motor_state.velocity) > motor1.safety_limits.cutoff_speed
            if (speed_cutoff_condition):
                warnings.warn("Cutoff velocity exceeded, system exiting...")
                rcpy.set_state(rcpy.EXITING)

            '''
            # Temperature Safety Cutoffs
            temperature_cutoff_condition = motor1.motor_state.temperature > motor1.safety_limits.cutoff_temperature or motor2.motor_state.temperature > motor2.safety_limits.cutoff_temperature
            if (temperature_cutoff_condition):
                warnings.warn("Cutoff temperature exceeded, system exiting...")
                rcpy.set_state(rcpy.EXITING)

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

            # PRETENSION BLOCK (only executes once)
            if is_system_pretensioned == False:
                # motor1
                v_des_m1 = -0.5
                #t_cable = np.abs(t1_cur_filt) / PULLEY_R 
                t_cable=force_cur
                t_ff_m1 = 0
                motor1.set_ref_velocity(v_des_m1, t_ff_m1)
                print("t_cable: ", t_cable)

                if t_cable > cable_pretension_setpoint:
                    is_system_pretensioned = True
                    # Set motor control type: "trajectory", "torque", "velocity"
                    motor1.set_control_mode(control_mode_motor1, self.sys_state.kp_m1, self.sys_state.kd_m1)
                    motor1.set_control_mode(control_mode_motor1, self.sys_state.kp_m1, self.sys_state.kd_m1)
                    
                    # TRIGGER HIGH
                    os.popen('echo out > /sys/class/gpio/gpio105/direction')
                    os.popen('echo 1 > /sys/class/gpio/gpio105/value')
                    os.popen('echo 0 > /sys/class/gpio/gpio105/value')
                    #os.popen('config-pin P8_18 gpio_pd')
                    
                    with lock:
                        self.sys_state.trigger = 1
                    
                    
            else: # MAIN CONTROL LOOP AFTER PRETENSION ACHIEVED
                                
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

                    force_des = compute_gravity_assistance(sh_el_cur)
                    r_pulley = PULLEY_R
                    #t_ff_m1 = force_des * r_pulley
                    
                    t_ff_m1 = compute_friction_compensation(force_des, sh_el_cur, sh_el_dot_cur, v_cur_m1)
                    #t_ff_m1 = compute_inertial_compensation(t_ff_m1, a1_acc_filt)

                    # Reverse direction of output on new box
                    t_ff_m1 = -1.0* t_ff_m1

                    # Send command to motor
                    motor1.set_ff_torque(t_ff_m1)

            # Save desired state to system
            with lock:
                # motor1 desired state (for logging)
                self.sys_state.p_des_m1 = p_des_m1
                self.sys_state.v_des_m1 = v_des_m1
                self.sys_state.t_ff_m1 = t_ff_m1

                self.sys_state.force_des = force_des

            time.sleep(max(next_time_instant-time.perf_counter(),0))


        # Force P8_08 LOW
        os.popen('echo 0 > /sys/class/gpio/gpio105/value')
        
        with lock:
            self.sys_state.trigger = 0
        
        print("Disabling motor!")
        ii=0
        while ii<4:
            print('Attempt'+ str(ii+1))
            motor1.stop_and_disable()
            ii+=1
            time.sleep(0.5)
        print('M1 disabled!')
            
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
        self.imu_fs = 200 # Need to read faster than IMU update frequency
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
            #sh_el_dot_back = imus[self.upperarm_id].compute_sh_elev_dot_back(sh_el)

            with lock:
                self.system_state.sh_el = sh_el
                self.system_state.sh_el_dot = sh_el_dot_gyro
                
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
        file_header = file_header + "a_m1," # Motor1 acceleration
        file_header = file_header + "kp_m2,kd_m2,p_des_m2,v_des_m2,t_des_m2," # Motor2 desired state
        file_header = file_header + "p_cur_m2,v_cur_m2,t_cur_m2," # Motor2 actual state
        file_header = file_header + "a_m2," # Motor1 acceleration
        file_header = file_header + "sh_el,sh_el_dot," # IMU
        file_header = file_header + "force_des,force_cur," # Load cell
        file_header = file_header + "trigger,gravity_support_level\n" # trigger state and gravity support level
        F_OUT.write(file_header)

        while rcpy.get_state() != rcpy.EXITING:
            next_time_instant = time.perf_counter() + (1.0 / self.save_fs)
            t = time.perf_counter()-t0

            with lock:
                data = "{:.5f},{:.2f},".format(t, self.sys_state.v_bat)
                data = data + "{:.2f},{:.2f},{:.3f},{:.3f},{:.3f},".format(self.sys_state.kp_m1,self.sys_state.kd_m1,self.sys_state.p_des_m1,self.sys_state.v_des_m1,self.sys_state.t_ff_m1)
                data = data + "{:.3f},{:.3f},{:.3f},".format(self.sys_state.motor1_state.position,self.sys_state.motor1_state.velocity,self.sys_state.motor1_state.torque)
                data = data + "{:.3f},".format(self.sys_state.acc_m1)
                #data = data + "{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},".format(self.sys_state.kp_m2,self.sys_state.kd_m2,self.sys_state.p_des_m2,self.sys_state.v_des_m2,self.sys_state.t_ff_m2)
                #data = data + "{:.3f},{:.3f},{:.3f},".format(self.sys_state.motor2_state.position,self.sys_state.motor2_state.velocity,self.sys_state.motor2_state.torque)
                #data = data + "{:.3f},".format(self.sys_state.acc_m2)
                data = data + "{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},".format(0,0,0,0,0)#To keep positions occupied by M2
                data = data + "{:.3f},{:.3f},{:.3f},".format(0,0,0)#To keep positions occupied by M2
                data = data + "{:.3f},".format(0)#To keep positions occupied by M2
                data = data + "{:.3f},{:.3f},".format(self.sys_state.sh_el,self.sys_state.sh_el_dot)
                data = data + "{:.3f},{:.3f},".format(self.sys_state.force_des,self.sys_state.force_cur)
                data = data + "{:.3f},{:.2f}\n".format(self.sys_state.trigger,self.sys_state.gravity_support_level)
            
            # Write can happen after lock is given up
            F_OUT.write(data)

            time.sleep(max(next_time_instant-time.perf_counter(),0))

        # Close the file once the thread ends
        F_OUT.close()
        
def main():        
    system_state = systemState()
    
    with lock:
        system_state.trigger = 0
    
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
        os.popen('echo 1 > /sys/class/gpio/gpio105/value')
        with lock:
            self.sys_state.trigger = 0
        os.popen('echo 0 > /sys/class/gpio/gpio105/value')
        
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

#from communication import motorCommunication
from communication import imuCommunication as ic
#from communication import loadCellCommunication as lcc

import digital_filters as filt

from scipy import signal
from scipy import interpolate

from collections import deque

import numpy as np
import time
import warnings

import ifcfg # Linux
import socket # Windows

class SafetyLimits:
    def __init__(self):
        self.cutoff_torque = 4.0 # Nm at motor output
        self.cutoff_speed = 25.0 # rad/s at motor output
        # TODO: ensure that temperature conversion is correct...
        self.cutoff_temperature = 35.0 # Deg C
        self.cutoff_positive_pos = 12.5 # Initialize position cutoffs to max
        self.cutoff_negative_pos = -12.5

# TODO: Use this class to hold the most recent updated motor state from the CAN bus.
#       Does it make sense to also have a class for the desired / commanded motor state.
class MotorState:
    def __init__(self):
        self.can_id = 0
        self.position = 0
        self.velocity = 0
        self.torque = 0
        self.torque_filt = 0
        self.temperature = 0
        self.error_code = 0      

class ControlState:
    def __init__(self):
        self.kp = 0
        self.kd = 0
        self.p_des = 0
        self.v_des = 0
        self.t_ff = 0
        self.control_mode = "none"
        
class Motor:
    def __init__(self, CAN_ID, FS, model, joint):
        
        
        # Model
        self.model = model
        
        # Main control loop frequency
        self.fs = FS
        self.dt = 1. / FS
        
        # Joint assigned
        self.joint = joint
        
        # Safety limits for motor
        self.safety_limits = SafetyLimits()

        # This object should always contain the most recently updated motor state.
        self.motor_state = MotorState()
        self.control_state = ControlState()

        self.a_cur = 0.0
        filter_order = 2
        filter_cutoff = 20
        sos = signal.butter(filter_order, filter_cutoff, 'lowpass', fs = self.fs, output = 'sos')
        self.vel_filter = filt.LiveSosFilter(sos)
        self.vel_filtered_vals = deque([0.0] * 4, maxlen = 4) # 4 needed for backwards differentiation

        self.can_id = CAN_ID
        self.mc = motorCommunication(CAN_ID = CAN_ID, model = model)

        # SOS filter for the motor torque readings
        filter_order = 3
        filter_cutoff = 3
        self.sos = signal.butter(filter_order, filter_cutoff, 'lowpass', fs = self.fs, output = 'sos')
        self.motor_torque_filter = filt.LiveSosFilter(self.sos)

    def enable(self):
        self.mc.enable()
        time.sleep(0.01)

    def disable(self):
        self.mc.disable()
        time.sleep(0.01)

    def stop(self):
        self.mc.stop()
        time.sleep(0.01)

    def enable_and_stop(self):
        self.mc.enable()
        time.sleep(0.01)
        self.mc.stop()
        time.sleep(0.01)
        
    def stop_and_disable(self):
        self.mc.stop()
        time.sleep(0.01)
        self.mc.disable()
        time.sleep(0.01)
    
    def tare_position(self):
        self.mc.tare()
        time.sleep(2)

    def set_cutoff_torque(self, cutoff_torque):
        self.safety_limits.cutoff_torque = cutoff_torque

    def set_cutoff_speed(self, cutoff_speed):
        self.safety_limits.cutoff_speed = cutoff_speed

    def set_cutoff_temperature(self, cutoff_temperature):
        self.safety_limits.cutuff_temperature = cutoff_temperature

    def set_control_gains(self, kp, kd):
        if (kp < 0.0):
            warnings.warn("Cannot set KP < 0.0, setting to zero.")
            kp = 0.0
        '''
        elif (kp > 50):
            warnings.warn("AK60-6 Motor does not like KP > 50.0, capping to 50.0")
            kp = 50.0
        '''
         
        if (kd < 0.0):
            warnings.warn("Cannot set KD < 0.0, setting to zero.")
            kd = 0.0
        '''
        elif (kd > 2.0):
            warnings.warn("AK60-6 Motor does not like KD > 2.0, capping to 2.0")
            kd = 2.0
        '''
        self.control_state.kp = kp
        self.control_state.kd = kd

    def get_control_gains(self):
        kp = self.control_state.kp
        kd = self.control_state.kd
        return kp, kd

    def set_control_mode(self, control_mode, kp, kd):
        self.control_state.control_mode = control_mode
        if control_mode == "trajectory":
            self.control_state.kp = kp
            self.control_state.kd = kd
        elif control_mode == "velocity":
            self.control_state.kp = 0.0
            self.control_state.kd = kd
        elif control_mode == "torque":
            self.control_state.kp = 0.0
            self.control_state.kd = 0.0
        else:
            print("Please insert a valid control mode value: trajectory, velocity, torque")
        print("Setting control mode for Motor {}: {}".format(self.can_id, control_mode))

    def set_ref_trajectory(self, p_des, v_des, t_ff):
        if self.control_state.control_mode == "trajectory":
            self.control_state.p_des = p_des
            self.control_state.v_des = v_des
            self.control_state.t_ff = t_ff
            self.mc.send_command(self.control_state.p_des, self.control_state.v_des, self.control_state.kp, self.control_state.kd, self.control_state.t_ff)
        else:
            print("Error: this method is only valid if in trajectory control mode")
        
    def set_ref_velocity(self, v_des, t_ff):
        if self.control_state.control_mode == "velocity":
            self.control_state.p_des = 0
            self.control_state.v_des = v_des
            self.control_state.t_ff = t_ff
            self.mc.send_command(self.control_state.p_des, self.control_state.v_des, self.control_state.kp, self.control_state.kd, self.control_state.t_ff)
        else:
            print("Error: this method is only valid if in velocity control mode")
        
    def set_ff_torque(self, t_ff):
        if self.control_state.control_mode == "torque":
            self.control_state.p_des = 0
            self.control_state.v_des = 0
            self.control_state.t_ff = t_ff
            self.mc.send_command(self.control_state.p_des, self.control_state.v_des, self.control_state.kp, self.control_state.kd, self.control_state.t_ff)
        else:
            print("Error: this method is only valid if in torque control mode")

    def update_filtered_torque(self, x):
        torque_filt = self.motor_torque_filter._process(x)
        self.motor_state.torque_filt = torque_filt
        return torque_filt


class Imu():
    rot_elem = 0
    receive_socket = 0
    send_socket = 0
    def __init__(self, receive_port, ip_address, send_port, axis_up):
        self.receive_port = receive_port
        self.ip_address = ip_address
        self.send_port = send_port
        self.axis_up = axis_up
        
        self.rot_matrix = [];
        self.gyro = [];

        self.sgol_data_window = []
        self.sgol_window_len = 11
        self.sgol_polyorder = 3
        self.sgol_deriv = 1 # Want the first derivative
        self.sgol_delta = 1.0 / 100.0 # TODO: Should match control loop frequency
        self.sgol_coeffs = []

        self.back_fd_x = [0.0] * 4
        self.back_fd_y = [0.0] * 4

        self.butter_raw = [0.0] * 3
        self.butter_filt = [0.0] * 3

        self.dsp1_x = deque([0.0] * 7, maxlen = 7)

        self.dsp2_x = deque([0.0] * 5, maxlen = 5)

        self.dt = 0.005 # TODO: Should match control loop frequency

    def initialize(self, dt):
        # Linux
        
        # BeagleBone Public IP address
        for name, interface in ifcfg.interfaces().items():
            if interface['device'] == "wlan0":      # Device name
                IPAddr = interface['inet']          # First IPv4 found
                print("You are connected to the NGIMU network. IP Address: ", IPAddr)  
                # Change IMUs UDP send addresses to match BeagleBone IP address
                self.set_imu_send_ip(IPAddr)
                print("Setting IMU send address to match the BeagleBone IP adress...")
        
        # Windows (next 3 lines)
        '''
        IPAddr = socket.gethostbyname(socket.gethostname())
        print("Detected local IP Address:", IPAddr)
        self.set_imu_send_ip(IPAddr)
        '''
        
        self.receive_socket = ic.init_receive_socket(self.receive_port)
        self.send_socket = ic.init_send_socket()
        
        if self.axis_up == 'X':
            self.rot_elem = 6
        elif self.axis_up == 'Y':
            self.rot_elem = 7
        elif self.axis_up == 'Z':
            self.rot_elem = 8
        else:
            self.rot_elem = 0
            self.gyro_elem = 0
            print("Error: please insert valid IMU orientation value (X, Y, Z)")

        self.dt = dt

        lp_butter_coeffs = signal.butter(2, 10, 'lowpass', fs = (1.0 / dt), output = 'ba')
        self.sh_el_filter = filt.LiveLFilter(b = lp_butter_coeffs[0], a = lp_butter_coeffs[1])
        self.sh_el_filtered_vals = deque([0.0] * 4, maxlen = 4)

        self.sgol_delta = dt
        self.sgol_data_window = [0.0] * self.sgol_window_len
        self.sgol_coeffs = signal.savgol_coeffs(self.sgol_window_len, self.sgol_polyorder, pos=self.sgol_window_len-1, deriv=self.sgol_deriv, delta=self.sgol_delta, use='dot')

    def identify(self):
        message_type = ic.identify
        message_value = 0
        ic.send_message(self.send_socket, self.ip_address, self.send_port, message_type, message_value)

    def set_imu_send_ip(self, send_ip):
        message_type = ic.wifi_send_ip
        message_value = send_ip
        ic.send_message(self.send_socket, self.ip_address, self.send_port, message_type, message_value)

    def read_imu(self): 
        messages = ic.read_messages(self.receive_socket)
        for message in messages:
            time_stamp = message[0]
            data_type = message[1]
            if data_type == "/matrix":
                rot_matrix = message[2:]    
            
        return rot_matrix

    def read_sh_elev_angle(self):        
        read_mat = False
        read_gyro = False
        while (read_mat & read_gyro) is False:
            messages = ic.read_messages(self.receive_socket)
            for message in messages:
                time_stamp = message[0]
                data_type = message[1]
                if data_type == "/matrix":
                    rot_matrix = message[2:]
                    sh_el = np.arccos(rot_matrix[self.rot_elem])
                    read_mat = True
                elif data_type == "/sensors":
                    gyro = [message[2], message[3], message[4]]
                    sh_el_gyro = gyro[2]/180*np.pi
                    read_gyro = True
                    
            '''
            message = messages[-1]
            time_stamp = message[0]
            data_type = message[1]
            if data_type == "/matrix":
                rot_matrix = message[2:]
                sh_el = np.arccos(rot_matrix[self.rot_elem])
                read = True
            '''
        return sh_el,sh_el_gyro
        
                
                 

    # NOTE: Seems to overestimate the derivative (when compared to non-causal filtering baseline)
    def compute_sh_elev_dot_sgol(self, sh_el):
        self.sgol_data_window.append(sh_el)
        self.sgol_data_window.pop(0)
        return self.sgol_coeffs.dot(self.sgol_data_window)

    # NOTE: Butterworth filter angle, and then do finite-differences on smoothened signal.
    #       Works quite robustly, but does add some delay. The best method so far.
    def compute_sh_elev_dot_back(self, sh_el_cur):
        sh_el_filt_cur = self.sh_el_filter.update_filter(sh_el_cur)
        self.sh_el_filtered_vals.appendleft(sh_el_filt_cur)

        # Source: https://web.media.mit.edu/~crtaylor/calculator.html
        f = self.sh_el_filtered_vals
        h = self.dt
        deriv = (-2*f[3]+9*f[2]-18*f[1]+11*f[0]) / (6*1.0*h**1)
        #deriv = (3*f[4]-16*f[3]+36*f[2]-48*f[1]+25*f[0]) / (12*1.0*h**1)
        #deriv = (-12*f[5]+75*f[4]-200*f[3]+300*f[2]-300*f[1]+137*f[0])/(60*1.0*h**1)
        return deriv

    # Source: https://www.dsprelated.com/showarticle/35.php
    def compute_sh_el_dot_dsp1(self, sh_el_cur):
        self.dsp1_x.appendleft(sh_el_cur)
        deriv = (-1.0 / 16.0) * self.dsp1_x[0] + self.dsp1_x[2] - self.dsp1_x[4] + (1.0 / 16.0) * self.dsp1_x[6]
        return deriv / 0.02

    # Source: https://www.dsprelated.com/showarticle/814.php
    def compute_sh_el_dot_dsp2(self, sh_el_cur):
        self.dsp2_x.appendleft(sh_el_cur)
        deriv = (-3.0/16.0) * self.dsp2_x[0] + (31.0/32.0) * self.dsp2_x[1] - (31.0/32.0) * self.dsp2_x[3] + (3.0/16.0) * self.dsp2_x[4]
        return deriv / 0.02

class loadCell():
    chan = 0

    def __init__(self, offset, gain):
        self.offset = offset
        self.gain = gain

    def initialize(self):
        self.chan = lcc.ADC_initialize()

    def read_force(self):
        force = (lcc.ADC_read_volt(self.chan)-self.offset)*self.gain
        return force

    def read_volt(self):
        volt = lcc.ADC_read_volt(self.chan)
        return volt

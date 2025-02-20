#!/usr/bin/python3
import can
#import tmotor_can_utils as tcan
#from can.interface import Bus
#from can import Message

#import osc_decoder
import socket

#import busio
#import board
#import adafruit_ads1x15.ads1115 as ADS
#from adafruit_ads1x15.analog_in import AnalogIn

import numpy as np

import binascii

from pythonosc import udp_client
'''
class motorCommunication():
    def __init__(self, CAN_ID, model):
        if model == "AK60_6":
            self.lim = tcan.AK60_6_LIMITS()
        elif model == "AK80_9":
            self.lim = tcan.AK80_9_LIMITS()
        else:
            print("WARNING: please insert a valid t-motor model. Default motor selected as AK60_6")
            self.lim = tcan.AK60_6_LIMITS()
            
        
        self.codes = tcan.TMOTOR_SPECIAL_CAN_CODES()
        self.CAN_ID = CAN_ID

        # CAN bus for sending messages
        self.bus = Bus(channel="can0", bustype="socketcan", bitrate=1000000)

        # TODO: This can go to tmotor_can_utils --> put them in class and create object in main
        self.ENABLE_MOTOR_MSG = tcan.tmotor_CAN_message(data = self.codes.ENABLE_CAN_DATA, CAN_ID = self.CAN_ID)
        self.DISABLE_MOTOR_MSG = tcan.tmotor_CAN_message(data = self.codes.DISABLE_CAN_DATA, CAN_ID = self.CAN_ID)
        self.TARE_MOTOR_MSG = tcan.tmotor_CAN_message(data = self.codes.ZERO_CAN_DATA, CAN_ID = self.CAN_ID)

        self.DATA_STOP_MOTOR = tcan.pack_cmd_data(0, 0, 0, 0, 0, self.lim)
        self.STOP_MOTOR_MSG = tcan.tmotor_CAN_message(data = self.DATA_STOP_MOTOR, CAN_ID = self.CAN_ID)

    def enable(self):
        self.bus.send(self.ENABLE_MOTOR_MSG)

    def disable(self):
        self.bus.send(self.DISABLE_MOTOR_MSG)

    def stop(self):
        self.bus.send(self.STOP_MOTOR_MSG)

    def tare(self):
        self.bus.send(self.TARE_MOTOR_MSG)

    def send_command(self, p_des, v_des, kp_des, kd_des, t_ff):
        data_cmd = tcan.pack_cmd_data(p_des, v_des, kp_des, kd_des, t_ff, self.lim)
        message_cmd = tcan.tmotor_CAN_message(data = data_cmd, CAN_ID = self.CAN_ID)
        # Send command
        try:
            self.bus.send(message_cmd)
        except Exception as e:
            print("Error during send: ", repr(e))
'''
class imuCommunication:
    reset = "/reset"
    sleep = "/sleep"
    identify = "/identify"
    apply_settings = "/apply"
    default = "/default"
    ahrs_zero = "/ahrs_zero"
    wifi_send_ip = "/wifi/send/ip"

    def init_send_socket():
        send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        return send_socket

    def init_receive_socket(receive_port):
        receive_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receive_socket.bind(("", receive_port))
        receive_socket.setblocking(True)
        receive_socket.settimeout(1)
        return receive_socket
        

    def send_message(send_socket, send_address, send_port, message_type, message_value):
        IMU_client = udp_client.SimpleUDPClient(send_address, send_port)
        IMU_client.send_message(message_type, message_value)
    
    def read_messages(receive_socket):
        messages = []
        try:
            data, addr = receive_socket.recvfrom(2048)
        except socket.error:
            print("Socket Error: please verify that you are connected to NGIMU wifi")
        except socket.timeout:
            print("UDP Timeout error")
        else:
            for message in osc_decoder.decode(data):
                messages.append(message)
        finally:
            return messages
'''
class loadCellCommunication:
    def ADC_initialize():
        i2c = busio.I2C(board.SCL, board.SDA)
        ads = ADS.ADS1115(i2c)
        ads.gain = 1
        chan = AnalogIn(ads, ADS.P0, ADS.P1)
        return chan

    def ADC_read_volt(chan):
        return chan.voltage
  '''
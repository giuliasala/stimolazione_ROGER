#!/usr/bin/python3

import numpy as np

from can import Message

# IMPORTANT: Make sure that these match motor configuration!
class AK60_6_LIMITS:
    P_MIN = -12.5
    P_MAX = 12.5
    V_MIN = -45.0
    V_MAX = 45.0
    T_MIN = -15.0
    T_MAX = 15.0
    KP_MIN = 0.0
    KP_MAX = 500.0
    KD_MIN = 0.0
    KD_MAX = 5.0
    
class AK80_9_LIMITS:
    P_MIN = -12.5
    P_MAX = 12.5
    V_MIN = -50.0
    V_MAX = 50.0
    T_MIN = -18.0
    T_MAX = 18.0
    KP_MIN = 0.0
    KP_MAX = 500.0
    KD_MIN = 0.0
    KD_MAX = 5.0


class TMOTOR_SPECIAL_CAN_CODES:
    ENABLE_CAN_DATA  = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC] # Enables the motor
    DISABLE_CAN_DATA = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD] # Disables the motor
    ZERO_CAN_DATA    = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE] # Sets current position to zero

def tmotor_CAN_message(data, CAN_ID):
    msg = Message(data=data)
    msg.is_extended_id = False
    msg.arbitration_id = CAN_ID
    msg.is_rx = False
    return msg

def pack_cmd_data(p_des: float, v_des: float, k_P: float, k_D: float, t_FF: float, lim):
    '''
    Package command data to send over CAN bus

    Arguments:
        p_des: Desired position
        v_des: Desired velocity
        k_P  : Proportional gain
        k_D  : Derivative gain
        t_FF : Feed-forward torque 
    Returns:
        A list with 8 properly formatted bytes to send over the CAN bus to the motor
    '''

    # Clipping
    p_des = np.clip(p_des, lim.P_MIN, lim.P_MAX)
    v_des = np.clip(v_des, lim.V_MIN, lim.V_MAX)
    t_FF  = np.clip(t_FF, lim.T_MIN, lim.T_MAX)
    k_P   = np.clip(k_P, lim.KP_MIN, lim.KP_MAX)
    k_D   = np.clip(k_D, lim.KD_MIN, lim.KD_MAX)

    # Convert to uints
    p_int = float_2_uint(p_des, lim.P_MIN, lim.P_MAX, 16)
    v_int = float_2_uint(v_des, lim.V_MIN, lim.V_MAX, 12)
    t_int = float_2_uint(t_FF, lim.T_MIN, lim.T_MAX, 12)
    kp_int = float_2_uint(k_P, lim.KP_MIN, lim.KP_MAX, 12)
    kd_int = float_2_uint(k_D, lim.KD_MIN, lim.KD_MAX, 12)

    '''
    print("p_des (int): {:d}, (hex) = {:04X}".format(p_int, p_int))
    print("v_des (int): {:d}, (hex) = {:03X}".format(v_int, v_int))
    print("kp_des (int): {:d}, (hex) = {:03X}".format(kp_int, kp_int))
    print("kd_des (int): {:d}, (hex) = {:03X}".format(kd_int, kd_int))
    print("t_des (int): {:d}, (hex) = {:03X}".format(t_int, t_int))
    '''
    data = [0] * 8

    data[0] = p_int >> 8                           # Position 8-H
    data[1] = p_int & 0xFF                         # Position 8-L
    data[2] = v_int >> 4                           # Velocity 8-H
    data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8) # Velocity 4-L and KP 4-H
    data[4] = kp_int & 0xFF                        # KP 8-L
    data[5] = kd_int >> 4                          # KD 8-H
    data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8) # KD 4-L and Torque 4-H
    data[7] = t_int & 0xFF                         # Torque 8-L

    '''
    for i in range(len(data)):
        print("{:d}: {:02X}".format(i, data[i]))
    '''

    return data

def unpack_reply(data, can_IDs, models):
    # Unpack from data buffer
    # TO DO: add error if can_id does not belong to can_IDs list so that data is not mistakenly saved.
    motor_flag = False
    can_id = data[0]
    
    idx = 0
    for motor_can_ID in can_IDs:
        if can_id == motor_can_ID:
            model = models[idx]
            motor_flag = True 

    if motor_flag:
        if model == "AK60_6":
            lim = AK60_6_LIMITS()   
        elif model == "AK80_9":
            lim = AK80_9_LIMITS()    
        else:
            model = "AK60_6"
            print("WARNING: unknown can ID. Default model selected AK60_6.")
    else:
        model = "AK60_6"
        print("WARNING: unknown can ID. Default model selected AK60_6.")
    
    p_int = (data[1] << 8) | data[2]
    v_int = (data[3] << 4) | (data[4] >> 4)
    t_int = ((data[4] & 0xF) << 8) | data[5] 
    temp = data[6]
    err = data[7]

    p_cur = uint_2_float(p_int, lim.P_MIN, lim.P_MAX, 16)
    v_cur = uint_2_float(v_int, lim.V_MIN, lim.V_MAX, 12)
    t_cur = uint_2_float(t_int, lim.T_MIN, lim.T_MAX, 12)

    # According to the documentation, the range should be -20 to 127
    # but this gives temperature values that don't make sense.
    # Comparing the integers to GUI values, I found that they convert
    # with a range of -40 to 215 (which is just equivalent to 
    # subtracting 40 from the uint8)
    temp = uint_2_float(temp, -40, 215, 8)
    
    #print("p_cur: {:2.2f}, v_cur: {:2.2f}, t_cur: {:2.2f}".format(p_cur, v_cur, t_cur))

    return can_id, p_cur, v_cur, t_cur, temp, err

# Cast a float to an unsigned integer
def float_2_uint(x: float, x_min: float, x_max: float, n_bits: int):
    '''
    Converts a float to an unsigned integer
    Arguments:
        x: a float
        x_min: minimum value (will clip to this if below)
        x_max: maximum value (will clip to this if above)
        n_bits: number of bits of unsigned int
    Returns:
        A positive integer between 0 and (2^n_bits - 1)
    '''
    span = x_max - x_min
    if (x < x_min): x = x_min
    elif (x > x_max): x = x_max
    x_casted = (x-x_min) * float(((1 << n_bits) - 1) / span)
    return int(x_casted)

def uint_2_float(x_int: int, x_min: float, x_max: float, n_bits: int):
    '''
    Converts an unsigned integer back to a float
    Arguments:
        x_int: an unsigned int
        x_min: minimum value 
        x_max: maximum value 
        n_bits: number of bits of the unsigned int
    Returns:
        A floating point represenation of x_int
    '''
    span = x_max - x_min
    offset = x_min
    x_float = float(x_int) * span / float((1 << n_bits)-1) + offset
    return x_float

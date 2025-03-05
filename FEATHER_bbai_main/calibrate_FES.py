#!/usr/bin/python3

from rehamove import *

import time
import keyboard
import os

import utils

def calibrate_rehamove(port_name, channel, filename):

    try:
        # Open USB port
        r = Rehamove(port_name)

        # Error handling for connecting to port while script is running
        retries = 20
        while r.rehamove is None:
            print("Rehamove device not detected. Retrying...")
            time.sleep(0.5)
            r = Rehamove(port_name)
            retries -= 1
        if r.rehamove is None:
            raise Exception("Failed to connect to Rehamove device after multiple attempts.")
        
        print(f"Version: {r.version()}, Battery: {r.battery()}%, Mode Info: {r.info()}")

        r.change_mode(1)     # Change to mid level (0-low, 1-mid)

        # Set frequency and duration of contraction
        freq = 40    # Hz
        period = 1/freq * 1000    # ms
        duration = 0.5     # s
        
        # Set parameters
        pw = 400     # us
        current = 0     # mA
        max_current = 30

        thresholds = {
            'tingling_current': None,
            'movement_current': None,
            'full_range_current': None,
            'pain_current': None,
        }

        print("Press 'j' when tingling is detected, 'k' for movement, 'l' for full range, 'p' for pain.")

        while current <= max_current:

            try:
                r.set_pulse(current, pw)
                r.start(channel, period)
                time.sleep(duration)

            except Exception as e:
                print(f"Error running pulse: {e}")
                continue
            
            # Interacting with user
            
            if keyboard.is_pressed('j') and thresholds['tingling_current'] is None:
                thresholds['tingling_current'] = current

            elif keyboard.is_pressed('k') and thresholds['movement_current'] is None:
                thresholds['movement_current'] = current
                
            elif keyboard.is_pressed('l') and thresholds['full_range_current'] is None:
                thresholds['full_range_current'] = current
            
            elif keyboard.is_pressed('p') and thresholds['pain_current'] is None:
                thresholds['pain_current'] = current
                
            if thresholds['pain_current'] is not None:
                break

            r.update()

            # Increment current and pulse width
            current += 0.5

        r.end()

        # If pain was detected before recording the full range, manually assign it as the value for pain - 1 iteration
        if thresholds['full_range_current'] is None:
            thresholds['full_range_current'] = thresholds['pain_current'] - 0.5

        # Save the data in a csv file
        print("Saving calibration data...")

        utils.save_to_json(filename, thresholds)
        print("Calibration data saved successfully.")

    except Exception as e:
        print(f"\nAn error occurred during calibration: {e}")    

if __name__ == '__main__':
    
    port_name = "COM7" # Windows
    #port_name = "/dev/ttyUSB0" # Linux

    user = input("Your name: ").lower().strip()
    channel = input("Channel colour (white for anterior, black for middle): ").lower().strip()
    if channel == "white":
        filename = f"{user}_anterior_calibration_data.json"
    elif channel == "black":
        filename = f"{user}_middle_calibration_data.json"
    
    calibrate_rehamove(port_name, channel, filename)
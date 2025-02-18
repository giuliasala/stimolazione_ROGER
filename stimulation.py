from rehamove import *
import csv
import time
import keyboard
import pandas as pd

def save_to_csv(data):

    filename = 'calibration_data.csv'

    try:
        # Open the CSV file and append the data as a new row
        with open(filename, 'a', newline='') as csvfile:
            fieldnames = data.keys()
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

            # Write headers only if the file is empty (first run)
            if csvfile.tell() == 0:
                writer.writeheader()

            # Write the calibration data as a new row
            writer.writerow(data)
    except Exception as e:
        print(f"Error writing to CSV file: {e}")

def init_rehamove(port_name):

    # Open USB port (Windows)
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

    # Set the fixed parameters
    freq = 40    # Hz
    period = 1/freq * 1000    # ms
    pw = 400     # us

    return r, period, pw

def calibrate(port_name, channel):
    
    r, period, pw = init_rehamove(port_name)

    # Duration of contraction for each current value
    duration = 1     # s
    
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
    save_to_csv(thresholds)
    print("Calibration data saved successfully.")

def stimulate(port_name, channel):

    data = pd.read_csv("calibration_data.csv")
    movement = data['movement_current'].dropna().mean()
    full_range = data['full_range_current'].dropna().mean()

    r, period, pw = init_rehamove(port_name)

    # Duration of contraction
    duration = 1     # s
    
    min_current = movement     # mA
    max_current = full_range

    # To do: rampa che parte dal min e arriva al max. Durata? Come "distrubuisco" gli scalini?


if __name__ == '__main__':
    
    port_name = "COM7"
    possible_channels = ["red", "r", "blue", "b", "grey1", "gray1", "g1", "black", "grey2", "gray2", "g2", "white"]

    # Manual choice between calibration and stimulation
    while True:

        choice = input("Do you want to calibrate (c) or stimulate (s)?").lower.strip()
        channel = input("Please enter the channel colour or tipe 'quit': ").lower().strip()

        if channel in possible_channels:

            if choice == "c":
                calibrate(port_name, channel)
            elif choice == "s":
                stimulate(port_name, channel)
            
            break

        elif channel == "quit":
            print("Exiting program")
            break

        else:
            print(f"Invalid channel. Please select a channel from {possible_channels}")
from rehamove import *
import csv
import time
import keyboard

def calibrate_rehamove(port_name, channel):

    try:
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

        # Set frequency and duration of contraction
        freq = 30    # Hz
        period = 1/freq * 1000    # ms
        total_time = 1500     # ms
        
        # Set initial parameters
        current = 0     # mA
        pw = 150     # us
        max_current = 25
        max_pw = 400

        thresholds = {
            'tingling_current': None,
            'tingling_pw': None,
            'movement_current': None,
            'movement_pw': None,
            'full_range_current': None,
            'full_range_pw': None,
            'pain_current': None,
            'pain_pw': None
        }

        print("Press 'j' when tingling is detected, 'k' for movement, 'l' for full range, 'p' for pain.")

        while current <= max_current and pw <= max_pw:
            
            print(f"Testing for: {current} mA and {pw} us")

            try:
                r.set_pulse(current, pw)
                r.run(channel, period, total_time)
            except Exception as e:
                print(f"Error running pulse: {e}")
                continue
            
            # Interacting with user
            # In this code, when the key is pressed the value is stored. This means the values can be overwritten
            
            if keyboard.is_pressed('j'):
                thresholds['tingling_current'] = current
                thresholds['tingling_pw'] = pw
                print(f"Recorded tingling threshold: {current} mA, {pw} µs")

            if keyboard.is_pressed('k'):
                thresholds['movement_current'] = current
                thresholds['movement_pw'] = pw
                print(f"Recorded movement threshold: {current} mA, {pw} µs")
                
            if keyboard.is_pressed('l'):
                thresholds['full_range_current'] = current
                thresholds['full_range_pw'] = pw
                print(f"Recorded full range of motion: {current} mA, {pw} µs")
            
            if keyboard.is_pressed('p'):
                thresholds['pain_current'] = current
                thresholds['pain_pw'] = pw
                print(f"Recorded pain threshold: {current} mA, {pw} µs")

            if thresholds['pain_current'] is not None:
                break

            # Increment current and pulse width
            current += 0.5
            pw += 5

        # Save the data in a csv file
        print("Saving calibration data...")
        save_to_csv(thresholds)
        print("Calibration data saved successfully.")

    except Exception as e:
        print(f"\nAn error occurred during calibration: {e}")    

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

if __name__ == '__main__':
    
    port_name = "COM7"
    possible_channels = ["red", "r", "blue", "b", "grey1", "gray1", "g1", "black", "grey2", "gray2", "g2", "white"]

    # Ask the user for the channel
    while True:
        channel = input("Please enter the channel colour or tipe 'quit': ").lower().strip()
        if channel in possible_channels:
            calibrate_rehamove(port_name, channel)
            break        
        elif channel == 'quit':
            print("Exiting program")
            break
        else:
            print(f"Invalid channel. Please select a channel from {possible_channels}")

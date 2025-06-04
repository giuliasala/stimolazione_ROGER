#!/usr/bin/python3

import json
import os
import sys
import select
import termios
import tty

def save_to_json(filename, new_data, key=None):
    data = {}
    
    if os.path.exists(filename):
        # Load existing data from the file
        with open(filename, 'r') as file:
            try:
                data = json.load(file)
            except json.JSONDecodeError:
                data = {}
    else:
        # Create an empty dictionary if the file does not exist
        data = {}

    # Check if new_data is a dictionary or a number
    if isinstance(new_data, dict):
        # If it's a dictionary, update the existing data
        data.update(new_data)
    elif isinstance(new_data, (int, float)) and key is not None:
        # If it's a number, add it with the provided key
        data[key] = new_data
    else:
        raise ValueError("new_data should be a dictionary or a number with a provided key")

    # Save the updated data back to the file
    with open(filename, 'w') as file:
        json.dump(data, file, indent=4)

def load_from_json(filename, key):

    try:
        with open(filename, 'r') as file:
            data = json.load(file)
            return data.get(key, 0)  # Default to 0 if not found
    except Exception as e:
        print(f"Error loading calibration data: {e}")
        return 0
    
def get_key_nonblocking():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return None
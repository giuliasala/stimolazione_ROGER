from rehamove import *
import time

def main():
    # Open USB port (Windows)
    r = Rehamove("COM7")

    # Error handling for connecting to port while script is running
    while r.rehamove == None:
        r = Rehamove("COM7")
        time.sleep(0.5)
    
    r.version()     # To know the version of the library we are using
    r.battery()     # To know battery %
    r.info ()     # To know if we are in low or mid level mode

    r.change_mode(1)     # Change to mid level (0-low, 1-mid)
    r.info ()

    # Set parameters
    I = 22
    PW = 350
    freq = 30    # Frequency in Hz
    period = 1/freq * 1000    # ms
    total_time = 1500     # ms

    r.set_pulse(I, PW)
    r.run("red", period, total_time)

if __name__ == '__main__':
	main()
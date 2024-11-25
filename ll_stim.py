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

    # Set parameters
    I = 8
    PW = 200
    freq = 30    # Frequency in Hz
    t = 1/freq
    
    for i in range(0, 40):
        r.pulse("red", I, PW)     # Set channel, current (mA) and duration (us)
        time.sleep(t)     # One pulse every t seconds


if __name__ == '__main__':
	main()
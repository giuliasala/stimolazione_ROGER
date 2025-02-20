'''

Da fare:
- Leggere angolo da IMU in un thread separato (deve funzionare sempre)
- Quando l'angolo supera una certa soglia (da decidere, guardo come fa Elena), inizio a stimolare -> stim deve essere thread?
- Continuo a stimolare fino a quando l'angolo raggiunge la seconda soglia (max_angle, da calibrazione)

'''

import threading
import time
import numpy as np

from rehamove import *

import utils
from read_IMU import systemState, readImuLoop

# Thread Lock
lock = threading.Lock()

class FESControl(threading.Thread):
    def __init__(self, name, system_state, port_name, muscle):
        threading.Thread.__init__(self)
        self.name = name
        self.system_state = system_state
        self.muscle = muscle
        self.freq = 40
        self.period = 1/self.freq * 1000
        self.duration = 0.5
        self.pw = 400
        self.current = utils.load_from_json(self.filename, "movement_current")
        self.max_current = utils.load_from_json(self.filename, "full_range_current")
        self.device = Rehamove(port_name)
        self.min_sh_el = np.pi/12
        self.max_sh_el = utils.load_from_json(self.filename, "max_angle")

    def run(self):
        with lock:
            current_angle = np.degrees(self.system_state.sh_el)
        
        # Non si può fare perché la stimolazione blocca il codice. Da studiare come fare!
        if current_angle >= self.min_sh_el:
            while current_angle < self.max_sh_el and self.current <= self.max_current:
                try:
                    self.device.set_pulse(self.current, self.pw)
                    self.device.start(self.muscle, self.period)
                    time.sleep(self.duration)
                    self.device.update()

                    self.current += 0.5

                    with lock:
                        current_angle = np.degrees(self.system_state.sh_el)

                except Exception as e:
                    print(f"Error during stimulation: {e}")
                    break

            self.device.end()
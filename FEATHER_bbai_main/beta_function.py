import math
import matplotlib.pyplot as plt
import numpy as np

def beta_function(tingle_current, max_current, T, t):
        slope = 20 / T
        i = tingle_current + (max_current - tingle_current) * np.sqrt(1 / (1 + np.exp(-slope * (t - T/2))))
        return i

def step_beta_function(i):
        i_step = np.round(i * 2 + 1e-9) / 2 # Add a small bias to ensure rounding up for ties
        return i_step

if __name__ == "__main__":
        tingle_current = 3
        max_current = 22
        T = 2 # time for the movement

        sampling_interval = 0.025  # 40Hz (real-world time step)
        num_points = int(T / sampling_interval) + 1
        t = np.linspace(0, T, num_points)
        f = beta_function(tingle_current, max_current, T, t)
        f_step = step_beta_function(f)

        fig, ax = plt.subplots(figsize=(10, 6))
        ax.plot(t, f, label="Beta-function")
        ax.step(t, f_step, label="Real", color="red", where="post")
        ax.axhline(y=tingle_current, color="black", linestyle="--")
        ax.axhline(y=max_current, color="black", linestyle="--")
        ax.set_yticks([0, tingle_current, max_current])
        ax.set(xlim=(0, T), xlabel="time")
        ax.set(ylim=(0, max_current+1), ylabel="current")

        plt.show()
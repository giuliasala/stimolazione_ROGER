import matplotlib.pyplot as plt
import numpy as np

def beta_function(min_current, max_current, T, t):
        midpoint = T / 2
        steepness = (max_current - min_current) / T
        norm_sigmoid = 1 / (1 + np.exp(-steepness * (t - midpoint)))
        i = min_current + (max_current - min_current) * norm_sigmoid
        return i

def step_beta_function(i):
        i_step = np.round(i * 2 + 1e-9) / 2 # Add a small bias to ensure rounding up for ties
        return i_step

if __name__ == "__main__":
        min_current = 9
        max_current = 22
        T = 1.5 # time for the movement

        sampling_interval = 1/30  # 30Hz (real-world time step)
        num_points = int(T / sampling_interval) + 1
        t = np.linspace(0, T, num_points)
        f = beta_function(min_current, max_current, T, t)
        f_step = step_beta_function(f)

        fig, ax = plt.subplots(figsize=(10, 6))
        ax.plot(t, f, label="Beta-function")
        ax.step(t, f_step, label="Real", color="red", where="post")
        ax.axhline(y=min_current, color="black", linestyle="--")
        ax.axhline(y=max_current, color="black", linestyle="--")
        ax.set_yticks([min_current, max_current])
        ax.set(xlim=(0, T), xlabel="time")
        ax.set(ylim=(min_current-1, max_current+1), ylabel="current")

        plt.show()
import matplotlib.pyplot as plt
import pandas as pd

if __name__ == "__main__":
    csv_path = "log.csv"
    data = pd.read_csv(csv_path)

    column_x = "time"
    column_y1 = "sh_el_deg"
    column_y2 = "stim_curr"

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.plot(data[column_x], data[column_y1], label=column_y1, color='blue')
    ax.plot(data[column_x], data[column_y2], label=column_y2, color='green')
    ax.set_xlabel(column_x)
    ax.legend()

    plt.show()
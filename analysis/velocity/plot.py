import numpy as np
import matplotlib.pyplot as plt

plots = []

def add_plot_from_file(fname):
    data = np.genfromtxt(
        fname,
        delimiter=",",
        skip_header=1,
        names=["t", "v"])

    start_time = data["t"][0]
    data["t"] = [(x - start_time) / 10**9 for x in data["t"]]
    plots.append((fname, data))

if __name__ == "__main__":
    add_plot_from_file("data/8tapes.csv")
    add_plot_from_file("data/6tape.csv")
    add_plot_from_file("data/1tape.csv")

    plt.rcParams['figure.figsize'] = [12, 8]
    fig, axs = plt.subplots(len(plots))
    fig.tight_layout()

    for ax in axs:
        ax.set_ylim([5, 18])

    for idx, (fname, data) in enumerate(plots):
        max_val = np.max(data["v"])
        avg_val = np.average(data["v"])
        min_val = np.min(data["v"])
        diff = max_val - min_val

        axs[idx].set_title(f"{fname}, deviation=+{max_val-avg_val:.3f}/-{avg_val-min_val:.3f} km/h")
        axs[idx].plot(data["t"], data["v"], "black")
        axs[idx].plot(data["t"], np.repeat(avg_val, len(data["t"])), "r--")
        axs[idx].plot(data["t"], np.repeat(max_val, len(data["t"])), "g+")
        axs[idx].plot(data["t"], np.repeat(min_val, len(data["t"])), "b+")

    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (km/h)')
    plt.savefig("velocity.png")
    plt.show()

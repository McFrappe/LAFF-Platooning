import numpy as np
import matplotlib.pyplot as plt

MIN_FORWARD = 715000
MAX_FORWARD = 800000
STEP_SIZE = 5000
MAX_VELOCITY = 10
MIN_VELOCITY = 0
POLYFIT_ORDER = 3

plots = []

def add_plot_from_file(fname):
    data = np.genfromtxt(
        fname,
        delimiter=",",
        skip_header=1,
        names=["pwm", "velocity"])

    plots.append((fname, data))

if __name__ == "__main__":
    add_plot_from_file("data/vehicle1.csv")
    add_plot_from_file("data/vehicle1.csv")

    plt.rcParams["figure.figsize"] = [12, 8]
    fig, axs = plt.subplots(min(len(plots), 2))
    fig.tight_layout()

    for idx, (fname, data) in enumerate(plots):
        axs[idx].set_ylim([MIN_VELOCITY, MAX_VELOCITY])
        axs[idx].set_xlim([MIN_FORWARD, MAX_FORWARD])
        axs[idx].set_title(fname)
        axs[idx].scatter(data["pwm"], data["velocity"], c="black")
        z = np.polyfit(data["pwm"], data["velocity"], POLYFIT_ORDER)
        p = np.poly1d(z)
        xp = np.linspace(MIN_FORWARD, MAX_FORWARD, (MAX_FORWARD-MIN_FORWARD)//STEP_SIZE)
        axs[idx].plot(xp, p(xp), "r--")

    plt.xlabel("Velocity")
    plt.ylabel("PWM")
    plt.savefig("velocity_pwm_mapper.png")
    # plt.show()

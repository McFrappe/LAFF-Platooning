import sys
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt

MIN_FORWARD = 715000
MAX_FORWARD = 775000
STEP_SIZE = 5000
MAX_VELOCITY = 30
MIN_VELOCITY = 0
POLYFIT_ORDER = 4

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Please input file name.")
        sys.exit(1)

    data = np.genfromtxt(
        sys.argv[1],
        delimiter=",",
        skip_header=1,
        names=["pwm", "velocity"])

    mpl.rcParams["backend"] = "TkAgg"
    plt.rcParams["figure.figsize"] = [12, 8]
    plt.tight_layout()

    plt.ylim([MIN_VELOCITY, MAX_VELOCITY])
    plt.xlim([MIN_FORWARD, MAX_FORWARD])
    plt.scatter(data["pwm"], data["velocity"], c="black")
    z = np.polyfit(data["velocity"], data["pwm"], POLYFIT_ORDER)
    p = np.poly1d(z)
    steps = (MAX_FORWARD-MIN_FORWARD)//STEP_SIZE
    xp = np.linspace(MIN_VELOCITY, MAX_VELOCITY, steps)
    plt.plot(p(xp), xp, "r--")

    plt.ylabel("Velocity (km/h)")
    plt.xlabel("PWM")
    plt.grid()
    plt.title("Velocity PWM map")
    plt.show()

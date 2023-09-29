from src.platoon.platoon import Platoon
from src.common.plot import plot_speed, plot_position, plot_distances

def simulate(num_steps, num_vehicles):
    p = Platoon(num_vehicles)  # all vehicles are standing still in an imaginary position of 0

    # Each step is 10ms
    for s in range(num_steps):
        p.run(s)

    # Truck 0-100km/h takes ~20s
    # 20s/(10ms->0.01s) = 2000 steps
    # 100/2000 = 0.05 km/h gain per step (acceleration)
    speeds = p.get_speeds() # km/h
    # ((10ms/3600000)->h) * km/h = km => km/1000 = m
    positions = p.get_positions() # position is in meters (position 1 is 1m)
    distances = p.get_distances() # distance is in meters

    plot_speed(speeds, num_vehicles)
    plot_position(positions, num_vehicles)
    plot_distances(distances, num_vehicles)

if __name__ == "__main__":
    simulate(10000, 4)
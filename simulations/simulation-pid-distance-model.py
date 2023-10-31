from src.platoon.platoon_pid_distance import PlatoonPidDistanceTruckS1, PlatoonPidDistanceTruckS2, PlatoonPidDistanceTruckS3
from src.common.plot import plot_speed, plot_position, plot_distances

def simulate(num_tick, num_vehicles, scenario):
    match scenario:
        case 1:
            p = PlatoonPidDistanceTruckS1(num_vehicles)
            suffix = "pid-distance-model-s1"
        case 2:
            p = PlatoonPidDistanceTruckS2(num_vehicles)
            suffix = "pid-distance-model-s2"
        case 3:
            p = PlatoonPidDistanceTruckS3(num_vehicles)
            suffix = "pid-distance-model-s3"
        case _:
            p = PlatoonPidDistanceTruckS1(num_vehicles)
            suffix = "pid-distance-model-s1"

    # Each tick is 10ms
    for tick in range(num_tick):
        p.run(tick)

    # Truck 0-100km/h takes ~20s
    # 20s/(10ms->0.01s) = 2000 tick
    # 100/2000 = 0.05 km/h gain per tick (acceleration)
    speeds = p.get_speeds() # km/h
    # ((10ms/3600000)->h) * km/h = km => km/1000 = m
    positions = p.get_positions() # position is in meters (position 1 is 1m)
    distances = p.get_distances() # distance is in meters

    plot_speed(speeds, num_vehicles, f'plots/speeds-with-{suffix}.png')
    plot_position(positions, num_vehicles, f'plots/position-with-{suffix}.png')
    plot_distances(distances, num_vehicles, f'plots/distances-with-{suffix}.png')

if __name__ == "__main__":
    simulate(20000, 5, 3)

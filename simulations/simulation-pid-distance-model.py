from src.platoon.platoon_pid_distance import PlatoonPidDistanceTruckS1, PlatoonPidDistanceTruckS2, PlatoonPidDistanceTruckS3
from src.common.plot import plot_speed, plot_travel_distance, plot_distances, plot_position


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

    speeds = p.get_speeds()
    travel_distance = p.get_travel_distance()
    distances = p.get_distances()
    positions = p.get_positions()

    plot_speed(speeds, num_vehicles, f'plots/speeds-with-{suffix}.png')
    plot_travel_distance(travel_distance, num_vehicles, f'plots/travel-distance-with-{suffix}.png')
    plot_distances(distances, num_vehicles, f'plots/distances-with-{suffix}.png')
    plot_position(positions, num_vehicles, f'plots/positions-with-{suffix}.png')


if __name__ == "__main__":
    simulate(20000, 5, 3)

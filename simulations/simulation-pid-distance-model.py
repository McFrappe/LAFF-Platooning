import sys, getopt
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


def main(argv):
    num_ticks = 0
    num_vehicles = 0
    scenario = 1
    opts, args = getopt.getopt(argv,"ht:s:v:",["ticks=","scenario=","vehicles="])
    for opt, arg in opts:
        if opt == '-h':
            print ('simulation-pid-distance-model.py -t <number of ticks> -s <scenario> -v "number of vehicles"')
            sys.exit()
        elif opt in ("-t", "--ticks"):
            num_ticks = int(arg)
        elif opt in ("-s", "--scenario"):
            scenario = int(arg)
        elif opt in ("-v", "--vehicles"):
            num_vehicles = int(arg)

    simulate(num_ticks, num_vehicles, scenario)



if __name__ == "__main__":
    main(sys.argv[1:])

    # Examples
    #simulate(20000, 5, 1)
    #simulate(2000, 5, 2)
    #simulate(20000, 5, 3)

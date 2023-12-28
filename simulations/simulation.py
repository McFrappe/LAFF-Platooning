import numpy as np
import sys, getopt
from src.platoon.platoon_distance_pid import PlatoonPidDistanceS1, PlatoonPidDistanceS2, PlatoonPidDistanceS3, PlatoonPidDistanceS4
from src.platoon.platoon_distance_pid_increase import PlatoonPidIncreaseDistanceS1, PlatoonPidIncreaseDistanceS2, PlatoonPidIncreaseDistanceS3, PlatoonPidIncreaseDistanceS4
from src.platoon.platoon_bidirectional_pid import PlatoonBidirectionalPidS3, PlatoonBidirectionalPidS1, PlatoonBidirectionalPidS2
from src.platoon.platoon_bidirectional_state_space import PlatoonBidirectionalStateSpaceS1, PlatoonBidirectionalStateSpaceS2, PlatoonBidirectionalStateSpaceS3, PlatoonBidirectionalStateSpaceS4
from src.platoon.platoon_bidirectional_state_space_with_pid import PlatoonBidirectionalStateSpaceWithPidS1, PlatoonBidirectionalStateSpaceWithPidS2, PlatoonBidirectionalStateSpaceWithPidS3, PlatoonBidirectionalStateSpaceWithPidS4
from src.common.plot import plot_speed, plot_travel_distance, plot_distances, plot_position
from src.vehicle.vehicle_specs import truck, kit_car, rc_car
from src.common.constants import tick_in_s


def simulate(num_tick, num_vehicles, scenario, type, model, period):
    match (scenario, type, model):
        case (1,1,1):
            p = PlatoonPidDistanceS1(num_vehicles, truck, period)
            suffix = "pid-distance-model-s1-truck"
        case (2,1,1):
            p = PlatoonPidDistanceS2(num_vehicles, truck, period)
            suffix = "pid-distance-model-s2-truck"
        case (3,1,1):
            p = PlatoonPidDistanceS3(num_vehicles, truck, period)
            suffix = "pid-distance-model-s3-truck"
        case (4,1,1):
            p = PlatoonPidDistanceS4(num_vehicles, truck, period)
            suffix = "pid-distance-model-s4-truck"
        case (1,3,1):
            p = PlatoonPidDistanceS1(num_vehicles, rc_car, period)
            suffix = "pid-distance-model-s1-rc-vehicle"
        case (2,3,1):
            p = PlatoonPidDistanceS2(num_vehicles, rc_car, period)
            suffix = "pid-distance-model-s2-rc-vehicle"
        case (3,3,1):
            p = PlatoonPidDistanceS3(num_vehicles, rc_car, period)
            suffix = "pid-distance-model-s3-rc-vehicle"
        case (1,1,2):
            p = PlatoonBidirectionalPidS1(num_vehicles, truck, period)
            suffix = "bidirectional-pid-model-s1-truck"
        case (2,1,2):
            p = PlatoonBidirectionalPidS2(num_vehicles, truck, period)
            suffix = "bidirectional-pid-model-s2-truck"
        case (3,1,2):
            p = PlatoonBidirectionalPidS3(num_vehicles, truck, period)
            suffix = "bidirectional-pid-model-s3-truck"
        case (1,1,3):
            p = PlatoonBidirectionalStateSpaceS1(num_vehicles, truck, period)
            suffix = "bidirectional-state-space-model-s1-truck"
        case (2,1,3):
            p = PlatoonBidirectionalStateSpaceS2(num_vehicles, truck, period)
            suffix = "bidirectional-state-space-model-s2-truck"
        case (3,1,3):
            p = PlatoonBidirectionalStateSpaceS3(num_vehicles, truck, period)
            suffix = "bidirectional-state-space-model-s3-truck"
        case (1,3,3):
            p = PlatoonBidirectionalStateSpaceS1(num_vehicles, rc_car, period)
            suffix = "bidirectional-state-space-model-s1-rc-vehicle"
        case (2,3,3):
            p = PlatoonBidirectionalStateSpaceS2(num_vehicles, rc_car, period)
            suffix = "bidirectional-state-space-model-s2-rc-vehicle"
        case (3,3,3):
            p = PlatoonBidirectionalStateSpaceS3(num_vehicles, rc_car, period)
            suffix = "bidirectional-state-space-model-s3-rc-vehicle"
        case (4,1,3):
            p = PlatoonBidirectionalStateSpaceS4(num_vehicles, truck, period)
            suffix = "bidirectional-state-space-model-s4-truck"
        case (1,1,4):
            p = PlatoonBidirectionalStateSpaceWithPidS1(num_vehicles, truck, period)
            suffix = "bidirectional-state-space-with-pid-model-s1-truck"
        case (2,1,4):
            p = PlatoonBidirectionalStateSpaceWithPidS2(num_vehicles, truck, period)
            suffix = "bidirectional-state-space-with-pid-model-s2-truck"
        case (3,1,4):
            p = PlatoonBidirectionalStateSpaceWithPidS3(num_vehicles, truck, period)
            suffix = "bidirectional-state-space-with-pid-model-s3-truck"
        case (4,1,4):
            p = PlatoonBidirectionalStateSpaceWithPidS4(num_vehicles, truck, period)
            suffix = "bidirectional-state-space-with-pid-model-s4-truck"
        case (1,1,5):
            p = PlatoonPidIncreaseDistanceS1(num_vehicles, truck, period)
            suffix = "distance-pid-increase-model-s1-truck"
        case (2,1,5):
            p = PlatoonPidIncreaseDistanceS2(num_vehicles, truck, period)
            suffix = "distance-pid-increase-model-s2-truck"
        case (3,1,5):
            p = PlatoonPidIncreaseDistanceS3(num_vehicles, truck, period)
            suffix = "distance-pid-increase-model-s3-truck"
        case (4,1,5):
            p = PlatoonPidIncreaseDistanceS4(num_vehicles, truck, period)
            suffix = "distance-pid-increase-model-s4-truck"
        case (_,_,_):
            print("Unknown option")
            exit(1)
    print(suffix)

    # Each tick is 10ms
    for tick in range(num_tick):
        p.run(tick)

    speeds = p.get_speeds()
    travel_distance = p.get_travel_distance()
    distances = p.get_distances()
    positions = p.get_positions()
    references = p.get_references()

    plot_speed(speeds, num_vehicles, f'plots/speeds-with-{suffix}.png')
    plot_travel_distance(travel_distance, num_vehicles, f'plots/travel-distance-with-{suffix}.png')
    plot_distances((distances, references), num_vehicles, f'plots/distances-with-{suffix}.png')
    plot_position(positions, num_vehicles, f'plots/positions-with-{suffix}.png')


def main(argv):
    num_ticks = 0
    num_vehicles = 0
    scenario = 1
    type = 1
    model = 1
    period = tick_in_s
    opts, args = getopt.getopt(argv,"ht:s:v:y:m:p:",["ticks=","scenario=","vehicles=","type=","model=","period="])
    for opt, arg in opts:
        if opt == '-h':
            print ('simulation-pid-distance-model.py -t <number of ticks> -s <scenario> -v <number of vehicles> -y <vehicle type>')
            print ('Vehicle type:')
            print ('\t1: Truck')
            print ('\t2: DIY-kit vehicle (TODO)')
            print ('\t3: RC vehicle (TODO)')
            print ('Model to use:')
            print ('\t1: PID distance')
            print ('\t2: Bidirectional')
            sys.exit()
        elif opt in ("-t", "--ticks"):
            num_ticks = int(arg)
        elif opt in ("-s", "--scenario"):
            scenario = int(arg)
        elif opt in ("-v", "--vehicles"):
            num_vehicles = int(arg)
        elif opt in ("-y", "--type"):
            type = int(arg)
        elif opt in ("-m", "--model"):
            model = int(arg)
        elif opt in ("-p", "--period"):
            period = float(arg)
            if tick_in_s > period:
                print(f'period must be greater than {tick_in_s}')
                exit(1)
        else:
            print(f"unknown option: {opt}")

    simulate(num_ticks, num_vehicles, scenario, type, model, period)



if __name__ == "__main__":
    main(sys.argv[1:])

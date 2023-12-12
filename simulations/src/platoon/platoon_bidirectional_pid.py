from src.platoon.platoon import Platoon
from src.vehicle.vehicle import Vehicle
from src.vehicle.vehicle_bidirectional import VehicleBidirectional
from src.platoon.scenario.vehicle_leader_bidirectional_pid import VehicleLeaderBidirectionalPidS1, VehicleLeaderBidirectionalPidS2, VehicleLeaderBidirectionalPidS3
from src.vehicle.vehicle_specs import dummy_vehicle
import numpy as np


class PlatoonBidirectional(Platoon):
    def __init__(self, num_vehicles, vehicle_specs, period):
        Platoon.__init__(self, num_vehicles, vehicle_specs, period)

    def run(self, tick):
        speeds_each_run = np.array([])
        positions_each_run = np.array([])
        distances_each_run = np.array([])
        travel_distance_each_run = np.array([])

        # v_in_front (order -1 if no vehicle is in front)
        v_in_front = Vehicle(-1, dummy_vehicle, 1) # dummy vehicle

        for v in self.vehicles:
            speeds_each_run = np.append(speeds_each_run, v.update_speed(tick, self.vehicles[0].get_current_speed(), v_in_front.get_position()))
            travel_distance_each_run = np.append(travel_distance_each_run, v.update_travel_distance())
            positions_each_run = np.append(positions_each_run, v.update_position(self.vehicles[0].get_travel_distance()))
            distances_each_run = np.append(distances_each_run, v.update_distance(v_in_front.get_travel_distance()))
            v.update_min_distance()
            v_in_front = v

        self.speeds.append(speeds_each_run)
        self.travel_distance.append(travel_distance_each_run)
        self.distances.append(distances_each_run)
        self.positions.append(positions_each_run)

        return speeds_each_run, positions_each_run, distances_each_run


class PlatoonBidirectionalPidS1(PlatoonBidirectional):
    def __init__(self, num_vehicles, vehicle_specs, period):
        PlatoonBidirectional.__init__(self, num_vehicles, vehicle_specs, period)

    def init_vehicles(self, num_vehicles, vehicle_specs):
        self.vehicles.append(VehicleLeaderBidirectionalPidS1(order=0, init_speed=0, init_travel_distance=0, init_position=0, init_distance=0, vehicle_specs=vehicle_specs))

        for i in range(num_vehicles-1):
            self.vehicles.append(VehicleBidirectional(i+1, init_speed=0, init_travel_distance=0, init_position=0, init_distance=0, vehicle_specs=vehicle_specs, period=self.period))


class PlatoonBidirectionalPidS2(PlatoonBidirectional):
    def __init__(self, num_vehicles, vehicle_specs, period):
        PlatoonBidirectional.__init__(self, num_vehicles, vehicle_specs, period)

    def init_vehicles(self, num_vehicles, vehicle_specs):
        self.vehicles.append(VehicleLeaderBidirectionalPidS2(order=0, init_speed=60, init_travel_distance=2.17, init_position=0, init_distance=0, vehicle_specs=vehicle_specs))

        for i in range(num_vehicles-1):
            self.vehicles.append(VehicleBidirectional(order=i+1, init_speed=60, init_travel_distance=2.17*(-i), init_position=2.17*(-i+1), init_distance=2.17, vehicle_specs=vehicle_specs, period=self.period))


class PlatoonBidirectionalPidS3(PlatoonBidirectional):
    def __init__(self, num_vehicles, vehicle_specs, period):
        PlatoonBidirectional.__init__(self, num_vehicles, vehicle_specs, period)

    def init_vehicles(self, num_vehicles, vehicle_specs):
        self.vehicles.append(VehicleLeaderBidirectionalPidS3(order=0, init_speed=0, init_travel_distance=20, init_position=0, init_distance=0, vehicle_specs=vehicle_specs))

        for i in range(num_vehicles-1):
            self.vehicles.append(VehicleBidirectional(i+1, init_speed=0, init_travel_distance=0, init_position=0, init_distance=0, vehicle_specs=vehicle_specs, period=self.period))

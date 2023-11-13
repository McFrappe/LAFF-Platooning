from src.platoon.platoon import Platoon
from src.vehicle.vehicle_bidirectional import VehicleBidirectional
from src.vehicle.vehicle_specs import truck
from src.vehicle.vehicle import Vehicle
from src.vehicle.vehicle_specs import dummy_vehicle
import numpy as np


class VehicleLeaderTruckS3(Vehicle):
    def __init__(self, order, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs):
        Vehicle.__init__(self, order, vehicle_specs)
        self.speed = init_speed
        self.position = init_position
        self.distance = init_distance

        self.leader_accelerating = True
        self.leader_initial_accelerating = True


    def update_speed(self, tick):
        if int(self.speed) == 60: # may fail?
            self.leader_accelerating = False
            self.leader_initial_accelerating = False
        elif int(self.speed) == 40 and not self.leader_initial_accelerating: # may fail?
            self.leader_accelerating = True
        
        if self.leader_accelerating:
            desired_speed = self.speed + 0.01
        else: 
            desired_speed = self.speed - 0.01
        self.speed = self.calculate_valid_speed(desired_speed)

        return self.speed


class PlatoonBidirectionalTruckS3(Platoon):
    def __init__(self, num_vehicles):
        Platoon.__init__(self, num_vehicles, truck)

    def init_vehicles(self, num_vehicles, vehicle_specs):
        self.vehicles.append(VehicleLeaderTruckS3(order=0, init_speed=0, init_travel_distance=20, init_position=0, init_distance=0, vehicle_specs=vehicle_specs))

        for i in range(num_vehicles-1):
            self.vehicles.append(VehicleBidirectional(i+1, init_speed=0, init_travel_distance=0, init_position=0, init_distance=0, vehicle_specs=vehicle_specs))

    # should be called each tick
    def run(self, tick):
        speeds_each_run = np.array([])
        positions_each_run = np.array([])
        distances_each_run = np.array([])
        travel_distance_each_run = np.array([])

        # v_in_front (order -1 if no vehicle is in front)
        v_in_front = Vehicle(-1, dummy_vehicle) # dummy vehicle

        # FIXME: ugly as fuck
        first = True

        for v in self.vehicles:
            # the first vehicles only take a single arguments.
            if first:
                speeds_each_run = np.append(speeds_each_run, v.update_speed(tick))
            else:
                speeds_each_run = np.append(speeds_each_run, v.update_speed(tick, self.vehicles[0].get_current_speed(), v_in_front.get_position()))

            travel_distance_each_run = np.append(travel_distance_each_run, v.update_travel_distance())
            positions_each_run = np.append(positions_each_run, v.update_position(self.vehicles[0].get_travel_distance()))
            distances_each_run = np.append(distances_each_run, v.update_distance(v_in_front.get_travel_distance()))
            v.update_min_distance()
            v_in_front = v
            first = False

        self.speeds.append(speeds_each_run)
        self.travel_distance.append(travel_distance_each_run)
        self.distances.append(distances_each_run)
        self.positions.append(positions_each_run)

        return speeds_each_run, positions_each_run, distances_each_run


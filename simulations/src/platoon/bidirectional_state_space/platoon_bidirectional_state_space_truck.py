from src.platoon.bidirectional_state_space.platoon_bidirectional_state_space import PlatoonBidirectionalStateSpace
from src.vehicle.vehicle_bidirectional_state_space import VehicleBidirectionalStateSpace
from src.vehicle.vehicle import Vehicle
from src.vehicle.vehicle_specs import dummy_vehicle, truck
import numpy as np

class VehicleLeaderTruckS1(Vehicle):
    def __init__(self, order, vehicle_specs, init_speed, init_travel_distance, init_position, init_distance):
        Vehicle.__init__(self, order, vehicle_specs)
        self.speed = init_speed
        self.position = init_position
        self.distance = init_distance
        self.travel_distance = init_travel_distance

        self.leader_accelerating = True
        self.leader_initial_accelerating = True
        self.mass = 6000


    def update_speed(self, tick, leader_speed, relative_position_infront, momentum_infront, momentum_behind, delta_infront, delta_behind):
        desired_speed = self.speed

        if self.speed < 50 and tick < 6000: #self.max_speed/2:
            max_acceleration = self.vehicle_specs.get_max_acceleration_in_km_per_h_per_tick()
            desired_speed = self.speed + max_acceleration/2
        elif tick > 6000: # TODO: should change to when it actually should start slowing down
            max_deceleration = self.vehicle_specs.get_max_deceleration_in_km_per_h_per_tick()
            desired_speed = self.speed - max_deceleration/24

        self.speed = self.calculate_valid_speed(desired_speed)

        return self.speed

    def get_momentum(self):
        return self.speed * self.mass

    def get_delta(self):
        return 0 # should never be called according to my calculations

class PlatoonBidirectionalStateSpaceTruckS1(PlatoonBidirectionalStateSpace):
    def __init__(self, num_vehicles):
        PlatoonBidirectionalStateSpace.__init__(self, num_vehicles, truck)

    def init_vehicles(self, num_vehicles, vehicle_specs):
        self.vehicles.append(VehicleLeaderTruckS1(order=0, init_speed=0, init_travel_distance=0, init_position=0, init_distance=0, vehicle_specs=vehicle_specs))

        for i in range(num_vehicles-1):
            self.vehicles.append(VehicleBidirectionalStateSpace(order=i+1, num_followers=(num_vehicles-1), init_speed=0, init_travel_distance=0, init_position=0, init_distance=0, vehicle_specs=vehicle_specs))


class VehicleLeaderTruckS2(Vehicle):
    def __init__(self, order, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs):
        Vehicle.__init__(self, order, vehicle_specs)
        self.speed = init_speed
        self.position = init_position
        self.distance = init_distance
        self.travel_distance = init_travel_distance

        self.leader_accelerating = True
        self.leader_initial_accelerating = True
        self.mass = 6000

    def update_speed(self, tick, leader_speed, relative_position_infront, momentum_infront, momentum_behind, delta_infront, delta_behind):
        max_deceleration = self.vehicle_specs.get_max_deceleration_in_km_per_h_per_tick()
        desired_speed = self.speed - max_deceleration#/5*2
        self.speed = self.calculate_valid_speed(desired_speed)

        return self.speed

    def get_momentum(self):
        return self.speed*self.mass

    def get_delta(self):
        return 0 # should never be called according to my calculations


class PlatoonBidirectionalStateSpaceTruckS2(PlatoonBidirectionalStateSpace):
    def __init__(self, num_vehicles):
        PlatoonBidirectionalStateSpace.__init__(self, num_vehicles, truck)
        self.num_vehicles = num_vehicles

    def init_vehicles(self, num_vehicles, vehicle_specs):
        self.vehicles.append(VehicleLeaderTruckS2(order=0, init_speed=60, init_travel_distance=2.17, init_position=0, init_distance=0, vehicle_specs=vehicle_specs))

        for i in range(num_vehicles-1):
            self.vehicles.append(VehicleBidirectionalStateSpace(order=i+1, num_followers=(num_vehicles-1), init_speed=60, init_travel_distance=2.17*(-i), init_position=2.17*(-i+1), init_distance=2.17, vehicle_specs=vehicle_specs))


class VehicleLeaderTruckS3(Vehicle):
    def __init__(self, order, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs):
        Vehicle.__init__(self, order, vehicle_specs)
        self.speed = init_speed
        self.position = init_position
        self.distance = init_distance

        self.leader_accelerating = True
        self.leader_initial_accelerating = True
        self.mass = 6000

    def update_speed(self, tick, leader_speed, relative_position_infront, momentum_infront, momentum_behind, delta_infront, delta_behind):
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

    def get_momentum(self):
        return self.speed*self.mass

    def get_delta(self):
        return 0 # should never be called according to my calculations


class PlatoonBidirectionalStateSpaceTruckS3(PlatoonBidirectionalStateSpace):
    def __init__(self, num_vehicles):
        PlatoonBidirectionalStateSpace.__init__(self, num_vehicles, truck)
        self.num_vehicles = num_vehicles

    def init_vehicles(self, num_vehicles, vehicle_specs):
        self.vehicles.append(VehicleLeaderTruckS3(order=0, init_speed=0, init_travel_distance=20, init_position=0, init_distance=0, vehicle_specs=vehicle_specs))

        for i in range(num_vehicles-1):
            self.vehicles.append(VehicleBidirectionalStateSpace(i+1, num_followers=(num_vehicles-1), init_speed=0, init_travel_distance=0, init_position=0, init_distance=0, vehicle_specs=vehicle_specs))

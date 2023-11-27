from src.platoon.platoon import Platoon
from src.vehicle.vehicle_pid_distance import VehiclePidDistance
from src.vehicle.vehicle_specs import rc_car
from src.vehicle.vehicle import Vehicle
import numpy as np


class VehicleLeaderRcVehicleS1(Vehicle):
    def __init__(self, order, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs):
        Vehicle.__init__(self, order, vehicle_specs)
        self.speed = init_speed
        self.position = init_position
        self.distance = init_distance
        self.travel_distance = init_travel_distance

        self.leader_accelerating = True
        self.leader_initial_accelerating = True


    def update_speed(self, tick):
        desired_speed = self.speed

        if self.speed < 15 and tick < 1000: #self.max_speed/2:
            max_acceleration = self.vehicle_specs.get_max_acceleration_in_km_per_h_per_tick()
            desired_speed = self.speed + max_acceleration/2
        elif tick > 1000: # TODO: should change to when it actually should start slowing down
            max_deceleration = self.vehicle_specs.get_max_deceleration_in_km_per_h_per_tick()
            desired_speed = self.speed - max_deceleration/24

        self.speed = self.calculate_valid_speed(desired_speed)

        return self.speed


class PlatoonPidDistanceRcVehicleS1(Platoon):
    def __init__(self, num_vehicles):
        Platoon.__init__(self, num_vehicles, rc_car)


    def init_vehicles(self, num_vehicles, vehicle_specs):
        self.vehicles.append(VehicleLeaderRcVehicleS1(order=0, init_speed=0, init_travel_distance=0, init_position=0, init_distance=0, vehicle_specs=vehicle_specs))

        for i in range(num_vehicles-1):
            self.vehicles.append(VehiclePidDistance(i+1, init_speed=0, init_travel_distance=0, init_position=0, init_distance=0, vehicle_specs=vehicle_specs))


class VehicleLeaderRcVehicleS2(Vehicle):
    def __init__(self, order, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs):
        Vehicle.__init__(self, order, vehicle_specs)
        self.speed = init_speed
        self.position = init_position
        self.distance = init_distance
        self.travel_distance = init_travel_distance

        self.leader_accelerating = True
        self.leader_initial_accelerating = True


    def update_speed(self, tick):
        max_deceleration = self.vehicle_specs.get_max_deceleration_in_km_per_h_per_tick()
        desired_speed = self.speed - max_deceleration
        self.speed = self.calculate_valid_speed(desired_speed)

        return self.speed


class PlatoonPidDistanceRcVehicleS2(Platoon):
    def __init__(self, num_vehicles):
        Platoon.__init__(self, num_vehicles, rc_car)

    def init_vehicles(self, num_vehicles, vehicle_specs):
        max_speed = vehicle_specs.get_max_speed_in_km_per_h()
        self.vehicles.append(VehicleLeaderRcVehicleS2(order=0, init_speed=max_speed, init_travel_distance=2.17, init_position=0, init_distance=0, vehicle_specs=vehicle_specs))
        #self.vehicles.append(VehiclePidDistance(order=1, init_speed=60, init_travel_distance=0, init_position=2.17, init_distance=2.17, vehicle_specs=vehicle_specs))

        for i in range(num_vehicles-1):
            self.vehicles.append(VehiclePidDistance(order=i+1, init_speed=max_speed, init_travel_distance=2.17*(-i), init_position=2.17*(-i+1), init_distance=2.17, vehicle_specs=vehicle_specs))


class VehicleLeaderRcVehicleS3(Vehicle):
    def __init__(self, order, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs):
        Vehicle.__init__(self, order, vehicle_specs)
        self.speed = init_speed
        self.position = init_position
        self.distance = init_distance
        self.vehicle_specs = vehicle_specs

        self.leader_accelerating = True
        self.leader_initial_accelerating = True


    def update_speed(self, tick):
        max_speed = self.vehicle_specs.get_max_speed_in_km_per_h()
        if int(self.speed) == max_speed/2: # may fail?
            self.leader_accelerating = False
            self.leader_initial_accelerating = False
        elif int(self.speed) == max_speed/3 and not self.leader_initial_accelerating: # may fail?
            self.leader_accelerating = True
        
        if self.leader_accelerating:
            desired_speed = self.speed + 0.01
        else: 
            desired_speed = self.speed - 0.01
        self.speed = self.calculate_valid_speed(desired_speed)

        return self.speed


class PlatoonPidDistanceRcVehicleS3(Platoon):
    def __init__(self, num_vehicles):
        Platoon.__init__(self, num_vehicles, rc_car)

    def init_vehicles(self, num_vehicles, vehicle_specs):
        self.vehicles.append(VehicleLeaderRcVehicleS3(order=0, init_speed=0, init_travel_distance=10, init_position=0, init_distance=0, vehicle_specs=vehicle_specs))

        for i in range(num_vehicles-1):
            self.vehicles.append(VehiclePidDistance(i+1, init_speed=0, init_travel_distance=0, init_position=0, init_distance=0, vehicle_specs=vehicle_specs))


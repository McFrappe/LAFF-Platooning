from src.platoon.platoon_bidirectional import PlatoonBidirectional
from src.vehicle.vehicle_bidirectional_mpc import VehicleBidirectionalMpc
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
        self.mass = 1


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
        return 0

    def get_delta(self):
        return 0 # should never be called according to my calculations

class VehicleDummy(Vehicle):
    def __init__(self, order, vehicle_specs):
        Vehicle.__init__(self, order, vehicle_specs)

    def get_momentum(self):
        return 0  
    
    def get_delta(self):
        return 0 

class PlatoonBidirectionalMpcTruckS1(PlatoonBidirectional):
    def __init__(self, num_vehicles):
        PlatoonBidirectional.__init__(self, num_vehicles, truck)
        self.num_vehicles = num_vehicles

    def init_vehicles(self, num_vehicles, vehicle_specs):
        self.vehicles.append(VehicleLeaderTruckS1(order=0, init_speed=0, init_travel_distance=0, init_position=0, init_distance=0, vehicle_specs=vehicle_specs))

        for i in range(num_vehicles-1):
            self.vehicles.append(VehicleBidirectionalMpc(order=i+1, num_followers=(num_vehicles-1), init_speed=0, init_travel_distance=0, init_position=0, init_distance=0, vehicle_specs=vehicle_specs))

    def run(self, tick):
        speeds_each_run = np.array([])
        positions_each_run = np.array([])
        distances_each_run = np.array([])
        travel_distance_each_run = np.array([])

        # v_in_front (order -1 if no vehicle is in front)
        v_in_front = VehicleDummy(-1, dummy_vehicle) # dummy vehicle
        v_last = VehicleDummy(-2, dummy_vehicle) # dummy vehicle
        self.vehicles.append(v_last)

        for idx, v in enumerate(self.vehicles):
            if idx == self.num_vehicles:
                break

            v_behind =  self.vehicles[idx+1]
            speeds_each_run = np.append(speeds_each_run, v.update_speed(tick, 
                                                                        self.vehicles[0].get_current_speed(), 
                                                                        v_in_front.get_position(), 
                                                                        v_in_front.get_momentum(), 
                                                                        v_behind.get_momentum(), 
                                                                        v_in_front.get_delta(),
                                                                        v_behind.get_delta()))
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



class VehicleLeaderTruckS3(Vehicle):
    def __init__(self, order, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs):
        Vehicle.__init__(self, order, vehicle_specs)
        self.speed = init_speed
        self.position = init_position
        self.distance = init_distance

        self.leader_accelerating = True
        self.leader_initial_accelerating = True
        self.mass = 1

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
        return 0

    def get_delta(self):
        return 0 # should never be called according to my calculations


class PlatoonBidirectionalMpcTruckS3(PlatoonBidirectional):
    def __init__(self, num_vehicles):

        PlatoonBidirectional.__init__(self, num_vehicles, truck)
        self.num_vehicles = num_vehicles

    def init_vehicles(self, num_vehicles, vehicle_specs):
        self.vehicles.append(VehicleLeaderTruckS3(order=0, init_speed=0, init_travel_distance=20, init_position=0, init_distance=0, vehicle_specs=vehicle_specs))

        for i in range(num_vehicles-1):
            self.vehicles.append(VehicleBidirectionalMpc(i+1, num_followers=(num_vehicles-1), init_speed=0, init_travel_distance=0, init_position=0, init_distance=0, vehicle_specs=vehicle_specs))

    def run(self, tick):
        speeds_each_run = np.array([])
        positions_each_run = np.array([])
        distances_each_run = np.array([])
        travel_distance_each_run = np.array([])

        # v_in_front (order -1 if no vehicle is in front)
        v_in_front = VehicleDummy(-1, dummy_vehicle) # dummy vehicle
        v_last = VehicleDummy(-2, dummy_vehicle) # dummy vehicle
        self.vehicles.append(v_last)

        for idx, v in enumerate(self.vehicles):
            if idx == self.num_vehicles:
                break

            v_behind =  self.vehicles[idx+1]
            speeds_each_run = np.append(speeds_each_run, v.update_speed(tick, 
                                                                        self.vehicles[0].get_current_speed(), 
                                                                        v_in_front.get_position(), 
                                                                        v_in_front.get_momentum(), 
                                                                        v_behind.get_momentum(), 
                                                                        v_in_front.get_delta(),
                                                                        v_behind.get_delta()))
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


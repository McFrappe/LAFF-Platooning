from src.platoon.platoon import Platoon
from src.vehicle.vehicle import Vehicle
from src.vehicle.vehicle_bidirectional_state_space import VehicleBidirectionalStateSpace
from src.platoon.scenario.vehicle_leader_bidirectional_state_space import VehicleLeaderBidirectionalStateSpaceS1, VehicleLeaderBidirectionalStateSpaceS2, VehicleLeaderBidirectionalStateSpaceS3
from src.vehicle.vehicle_specs import dummy_vehicle
import numpy as np


class VehicleDummy(Vehicle):
    def __init__(self, order, vehicle_specs):
        Vehicle.__init__(self, order, vehicle_specs, 1)

    def get_momentum(self):
        return 0  
    
    def get_delta(self):
        return 0 


class PlatoonBidirectionalStateSpace(Platoon):
    def __init__(self, num_vehicles, vehicle_specs, period):
        Platoon.__init__(self, num_vehicles, vehicle_specs, period)
        self.num_vehicles = num_vehicles

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


class PlatoonBidirectionalStateSpaceS1(PlatoonBidirectionalStateSpace):
    def __init__(self, num_vehicles, vehicle_specs, period):
        PlatoonBidirectionalStateSpace.__init__(self, num_vehicles, vehicle_specs, period)

    def init_vehicles(self, num_vehicles, vehicle_specs):
        self.vehicles.append(VehicleLeaderBidirectionalStateSpaceS1(order=0, init_speed=0, init_travel_distance=0, init_position=0, init_distance=0, vehicle_specs=vehicle_specs))

        ss_par = vehicle_specs.get_state_space_vehicle_parameters()
        for i in range(num_vehicles-1):
            self.vehicles.append(VehicleBidirectionalStateSpace(order=i+1, num_followers=(num_vehicles-1), init_speed=0, init_travel_distance=0, init_position=0, init_distance=0, vehicle_specs=vehicle_specs, state_space_vehicle_parameters=ss_par, period=self.period))


class PlatoonBidirectionalStateSpaceS2(PlatoonBidirectionalStateSpace):
    def __init__(self, num_vehicles, vehicle_specs, period):
        PlatoonBidirectionalStateSpace.__init__(self, num_vehicles, vehicle_specs, period)

    def init_vehicles(self, num_vehicles, vehicle_specs):
        self.vehicles.append(VehicleLeaderBidirectionalStateSpaceS2(order=0, init_speed=60, init_travel_distance=2.17, init_position=0, init_distance=0, vehicle_specs=vehicle_specs))

        ss_par = vehicle_specs.get_state_space_vehicle_parameters()
        for i in range(num_vehicles-1):
            self.vehicles.append(VehicleBidirectionalStateSpace(order=i+1, num_followers=(num_vehicles-1), init_speed=60, init_travel_distance=2.17*(-i), init_position=2.17*(-i+1), init_distance=2.17, vehicle_specs=vehicle_specs,state_space_vehicle_parameters=ss_par, period=self.period))


class PlatoonBidirectionalStateSpaceS3(PlatoonBidirectionalStateSpace):
    def __init__(self, num_vehicles, vehicle_specs, period):
        PlatoonBidirectionalStateSpace.__init__(self, num_vehicles, vehicle_specs, period)

    def init_vehicles(self, num_vehicles, vehicle_specs):
        self.vehicles.append(VehicleLeaderBidirectionalStateSpaceS3(order=0, init_speed=0, init_travel_distance=20, init_position=0, init_distance=0, vehicle_specs=vehicle_specs))

        ss_par = vehicle_specs.get_state_space_vehicle_parameters()
        for i in range(num_vehicles-1):
            self.vehicles.append(VehicleBidirectionalStateSpace(i+1, num_followers=(num_vehicles-1), init_speed=0, init_travel_distance=0, init_position=0, init_distance=0, vehicle_specs=vehicle_specs, state_space_vehicle_parameters=ss_par, period=self.period))

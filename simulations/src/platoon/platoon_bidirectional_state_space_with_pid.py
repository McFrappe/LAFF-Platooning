from src.platoon.platoon import Platoon
from src.vehicle.vehicle import Vehicle
from src.vehicle.vehicle_bidirectional_state_space_with_pid import VehicleBidirectionalStateSpaceWithPid
from src.platoon.scenario.vehicle_leader_bidirectional_state_space import VehicleLeaderBidirectionalStateSpaceS1, VehicleLeaderBidirectionalStateSpaceS2, VehicleLeaderBidirectionalStateSpaceS3, VehicleLeaderBidirectionalStateSpaceS4
from src.platoon.platoon_bidirectional_state_space import PlatoonBidirectionalStateSpace
from src.vehicle.vehicle_specs import dummy_vehicle
import numpy as np

class PlatoonBidirectionalStateSpaceWithPidS1(PlatoonBidirectionalStateSpace):
    def __init__(self, num_vehicles, vehicle_specs, period):
        PlatoonBidirectionalStateSpace.__init__(self, num_vehicles, vehicle_specs, period)

    def init_vehicles(self, num_vehicles, vehicle_specs):
        self.vehicles.append(VehicleLeaderBidirectionalStateSpaceS1(order=0, init_speed=0, init_travel_distance=0, init_position=0, init_distance=0, vehicle_specs=vehicle_specs))

        ss_par = vehicle_specs.get_state_space_vehicle_parameters()
        for i in range(num_vehicles-1):
            self.vehicles.append(VehicleBidirectionalStateSpaceWithPid(order=i+1, num_followers=(num_vehicles-1), init_speed=0, init_travel_distance=0, init_position=0, init_distance=0, vehicle_specs=vehicle_specs, state_space_vehicle_parameters=ss_par, period=self.period))


class PlatoonBidirectionalStateSpaceWithPidS2(PlatoonBidirectionalStateSpace):
    def __init__(self, num_vehicles, vehicle_specs, period):
        PlatoonBidirectionalStateSpace.__init__(self, num_vehicles, vehicle_specs, period)

    def init_vehicles(self, num_vehicles, vehicle_specs):
        self.vehicles.append(VehicleLeaderBidirectionalStateSpaceS2(order=0, init_speed=60, init_travel_distance=2.17, init_position=0, init_distance=0, vehicle_specs=vehicle_specs))

        ss_par = vehicle_specs.get_state_space_vehicle_parameters()
        for i in range(num_vehicles-1):
            self.vehicles.append(VehicleBidirectionalStateSpaceWithPid(order=i+1, num_followers=(num_vehicles-1), init_speed=60, init_travel_distance=2.17*(-i), init_position=2.17*(-i+1), init_distance=2.17, vehicle_specs=vehicle_specs,state_space_vehicle_parameters=ss_par, period=self.period))


class PlatoonBidirectionalStateSpaceWithPidS3(PlatoonBidirectionalStateSpace):
    def __init__(self, num_vehicles, vehicle_specs, period):
        PlatoonBidirectionalStateSpace.__init__(self, num_vehicles, vehicle_specs, period)

    def init_vehicles(self, num_vehicles, vehicle_specs):
        self.vehicles.append(VehicleLeaderBidirectionalStateSpaceS3(order=0, init_speed=0, init_travel_distance=20, init_position=0, init_distance=0, vehicle_specs=vehicle_specs))

        ss_par = vehicle_specs.get_state_space_vehicle_parameters()
        for i in range(num_vehicles-1):
            self.vehicles.append(VehicleBidirectionalStateSpaceWithPid(i+1, num_followers=(num_vehicles-1), init_speed=0, init_travel_distance=0, init_position=0, init_distance=0, vehicle_specs=vehicle_specs, state_space_vehicle_parameters=ss_par, period=self.period))


class PlatoonBidirectionalStateSpaceWithPidS4(PlatoonBidirectionalStateSpace):
    def __init__(self, num_vehicles, vehicle_specs, period):
        PlatoonBidirectionalStateSpace.__init__(self, num_vehicles, vehicle_specs, period)

    def init_vehicles(self, num_vehicles, vehicle_specs):
        self.vehicles.append(VehicleLeaderBidirectionalStateSpaceS4(order=0, init_speed=0, init_travel_distance=0, init_position=0, init_distance=0, vehicle_specs=vehicle_specs))

        ss_par = vehicle_specs.get_state_space_vehicle_parameters()
        for i in range(num_vehicles-1):
            self.vehicles.append(VehicleBidirectionalStateSpaceWithPid(i+1, num_followers=(num_vehicles-1), init_speed=0, init_travel_distance=0, init_position=0, init_distance=0, vehicle_specs=vehicle_specs, state_space_vehicle_parameters=ss_par, period=self.period))


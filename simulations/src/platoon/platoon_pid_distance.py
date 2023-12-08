from src.platoon.platoon import Platoon
from src.platoon.scenario.vehicle_leader_distance_pid import VehicleLeaderDistancePidS1, VehicleLeaderDistancePidS2, VehicleLeaderDistancePidS3, VehicleLeaderDistancePidS4
from src.vehicle.vehicle_pid_distance import VehiclePidDistance
from src.vehicle.vehicle_specs import truck


class PlatoonPidDistanceS1(Platoon):
    def __init__(self, num_vehicles, vehicle_specs):
        Platoon.__init__(self, num_vehicles, vehicle_specs)

    def init_vehicles(self, num_vehicles, vehicle_specs):
        self.vehicles.append(VehicleLeaderDistancePidS1(order=0, init_speed=0, init_travel_distance=0, init_position=0, init_distance=0, vehicle_specs=vehicle_specs))

        for i in range(num_vehicles-1):
            self.vehicles.append(VehiclePidDistance(i+1, init_speed=0, init_travel_distance=0, init_position=0, init_distance=0, vehicle_specs=vehicle_specs))


class PlatoonPidDistanceS2(Platoon):
    def __init__(self, num_vehicles, vehicle_specs):
        Platoon.__init__(self, num_vehicles, vehicle_specs)

    def init_vehicles(self, num_vehicles, vehicle_specs):
        self.vehicles.append(VehicleLeaderDistancePidS2(order=0, init_speed=60, init_travel_distance=2.17, init_position=0, init_distance=0, vehicle_specs=vehicle_specs))

        for i in range(num_vehicles-1):
            self.vehicles.append(VehiclePidDistance(order=i+1, init_speed=60, init_travel_distance=2.17*(-i), init_position=2.17*(-i+1), init_distance=2.17, vehicle_specs=vehicle_specs))


class PlatoonPidDistanceS3(Platoon):
    def __init__(self, num_vehicles, vehicle_specs):
        Platoon.__init__(self, num_vehicles, vehicle_specs)

    def init_vehicles(self, num_vehicles, vehicle_specs):
        self.vehicles.append(VehicleLeaderDistancePidS3(order=0, init_speed=0, init_travel_distance=20, init_position=0, init_distance=0, vehicle_specs=vehicle_specs))

        for i in range(num_vehicles-1):
            self.vehicles.append(VehiclePidDistance(i+1, init_speed=0, init_travel_distance=0, init_position=0, init_distance=0, vehicle_specs=vehicle_specs))


class PlatoonPidDistanceS4(Platoon):
    def __init__(self, num_vehicles, vehicle_specs):
        Platoon.__init__(self, num_vehicles, vehicle_specs)

    def init_vehicles(self, num_vehicles, vehicle_specs):
        self.vehicles.append(VehicleLeaderDistancePidS4(order=0, init_speed=0, init_travel_distance=20, init_position=0, init_distance=0, vehicle_specs=vehicle_specs))

        for i in range(num_vehicles-1):
            self.vehicles.append(VehiclePidDistance(i+1, init_speed=0, init_travel_distance=0, init_position=0, init_distance=0, vehicle_specs=vehicle_specs))


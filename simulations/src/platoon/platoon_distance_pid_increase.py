from src.platoon.platoon import Platoon
from src.platoon.scenario.vehicle_leader_distance_pid import VehicleLeaderDistancePidS1, VehicleLeaderDistancePidS2, VehicleLeaderDistancePidS3, VehicleLeaderDistancePidS4
from src.vehicle.vehicle_distance_pid_increase import VehiclePidIncreaseDistance
from src.vehicle.vehicle_specs import truck


class PlatoonPidIncreaseDistanceS1(Platoon):
    def __init__(self, num_vehicles, vehicle_specs, period):
        Platoon.__init__(self, num_vehicles, vehicle_specs, period)

    def init_vehicles(self, num_vehicles, vehicle_specs):
        self.vehicles.append(VehicleLeaderDistancePidS1(order=0, init_speed=0, init_travel_distance=0, init_position=0, init_distance=0, vehicle_specs=vehicle_specs))

        for i in range(num_vehicles-1):
            self.vehicles.append(VehiclePidIncreaseDistance(i+1, init_speed=0, init_travel_distance=0, init_position=0, init_distance=0, vehicle_specs=vehicle_specs, period=self.period))


class PlatoonPidIncreaseDistanceS2(Platoon):
    def __init__(self, num_vehicles, vehicle_specs, period):
        Platoon.__init__(self, num_vehicles, vehicle_specs, period)

    def init_vehicles(self, num_vehicles, vehicle_specs):
        init_min_distance = .93
        self.vehicles.append(VehicleLeaderDistancePidS2(order=0, init_speed=60, init_travel_distance=init_min_distance, init_position=0, init_distance=0, vehicle_specs=vehicle_specs))

        for i in range(num_vehicles-1):
            self.vehicles.append(VehiclePidIncreaseDistance(order=i+1, init_speed=60, init_travel_distance=init_min_distance*(-i), init_position=init_min_distance*(-i+1), init_distance=init_min_distance, vehicle_specs=vehicle_specs, period=self.period))


class PlatoonPidIncreaseDistanceS3(Platoon):
    def __init__(self, num_vehicles, vehicle_specs, period):
        Platoon.__init__(self, num_vehicles, vehicle_specs, period)

    def init_vehicles(self, num_vehicles, vehicle_specs):
        self.vehicles.append(VehicleLeaderDistancePidS3(order=0, init_speed=0, init_travel_distance=0, init_position=0, init_distance=0, vehicle_specs=vehicle_specs))

        for i in range(num_vehicles-1):
            self.vehicles.append(VehiclePidIncreaseDistance(i+1, init_speed=0, init_travel_distance=0, init_position=0, init_distance=0, vehicle_specs=vehicle_specs, period=self.period))


class PlatoonPidIncreaseDistanceS4(Platoon):
    def __init__(self, num_vehicles, vehicle_specs, period):
        Platoon.__init__(self, num_vehicles, vehicle_specs, period)

    def init_vehicles(self, num_vehicles, vehicle_specs):
        self.vehicles.append(VehicleLeaderDistancePidS4(order=0, init_speed=0, init_travel_distance=0, init_position=0, init_distance=0, vehicle_specs=vehicle_specs))

        for i in range(num_vehicles-1):
            self.vehicles.append(VehiclePidIncreaseDistance(i+1, init_speed=0, init_travel_distance=0, init_position=0, init_distance=0, vehicle_specs=vehicle_specs, period=self.period))


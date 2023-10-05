from src.platoon.platoon import Platoon
from src.vehicle.vehicle_pid_distance import VehiclePidDistance
import numpy as np


class PlatoonPidDistance(Platoon):
    def __init__(self, num_vehicles):
        Platoon.__init__(self, num_vehicles)

    def init_vehicles(self, num_vehicles):
        self.vehicles.append(VehiclePidDistance(order=0, init_speed=0, init_position=0, init_distance=0))
        for i in range(num_vehicles-1):
            self.vehicles.append(VehiclePidDistance(i+1, init_speed=0, init_position=0, init_distance=0))

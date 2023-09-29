from src.vehicle.vehicle import Vehicle
import numpy as np


class Platoon:

    def __init__(self, num_vehicles) -> None:
        self.vehicles = []
        self.speeds = []
        self.positions = []
        self.distances = []

        self.init_vehicles(num_vehicles)

    def init_vehicles(self, num_vehicles):
        for i in range(num_vehicles):
            self.vehicles.append(Vehicle(i))

    # should be called each step
    def run(self, step):
        speeds_each_run = np.array([])
        positions_each_run = np.array([])
        distances_each_run = np.array([])
        # v_in_front (order -1 if no vehicle is in front)
        v_in_front = Vehicle(-1) # dummy vehicle

        for v in self.vehicles:
            speeds_each_run = np.append(speeds_each_run, v.update_speed(step))
            positions_each_run = np.append(positions_each_run, v.update_position())
            distances_each_run = np.append(distances_each_run, v.update_distance(v_in_front.get_position()))
            v_in_front = v

        self.speeds.append(speeds_each_run)
        self.positions.append(positions_each_run)
        self.distances.append(distances_each_run)

        return speeds_each_run, positions_each_run, distances_each_run

    def get_speeds(self):
        return self.speeds

    def get_positions(self):
        return self.positions

    def get_distances(self):
        return self.distances
from src.vehicle.vehicle import Vehicle
import numpy as np


class Platoon:

    def __init__(self, num_vehicles) -> None:
        self.vehicles = []
        self.speeds = []
        self.positions = []
        self.distances = []

        self.new_vehicle_last(order=0, init_speed=40, init_position=100, init_distance=0)
        # initiate all vehicles and add them to an array
        for i in range(num_vehicles-1):
            self.new_vehicle_last(order=i+1, init_speed=20, init_position=80, init_distance=20)

    def new_vehicle_last(self, order, init_speed, init_position, init_distance):
        self.vehicles.append(Vehicle(order, init_speed, init_position, init_distance))

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
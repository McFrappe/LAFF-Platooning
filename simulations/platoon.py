from vehicle import Vehicle
import numpy as np

class Platoon:

    def __init__(self, num_vehicles) -> None:
        self.vehicles = []
        self.distances = np.array([])
        # np array of speed, position, and distance

        # initiate all vehicles and add them to an array
        for order in range(num_vehicles):
            self.new_vehicle_last(order)


    def new_vehicle_last(self, order):
        self.vehicles.append(Vehicle(order))

    # should be called each step
    def run(self, step):
        speeds = np.array([])
        positions = np.array([])
        distances_each_run = np.array([])
        # v_in_front (order -1 if no vehicle is in front)
        v_in_front = Vehicle(-1) # dummy vehicle

        for v in self.vehicles:
            speeds = np.append(speeds, v.update_speed(step))
            positions = np.append(positions, v.update_position())
            distances_each_run = np.append(distances_each_run, v.update_distance(v_in_front.get_position()))
            v_in_front = v

        self.distances = np.append(self.distances, distances_each_run)
        return speeds, positions, distances_each_run


    def get_distances(self):
        return self.distances
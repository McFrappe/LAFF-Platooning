from src.vehicle.vehicle import Vehicle
from src.vehicle.vehicle_specs import dummy_vehicle
import numpy as np


class Platoon:
    def __init__(self, num_vehicles, vehicle_specs, period) -> None:
        self.vehicles = []
        self.speeds = []
        self.travel_distance = []
        self.distances = []
        self.positions = []
        self.references = []
        self.period = period

        self.init_vehicles(num_vehicles, vehicle_specs)

    def init_vehicles(self, num_vehicles, vehicle_specs):
        for i in range(num_vehicles):
            self.vehicles.append(Vehicle(i, vehicle_specs))

    # should be called each tick
    def run(self, tick):
        speeds_each_run = np.array([])
        positions_each_run = np.array([])
        distances_each_run = np.array([])
        reference_each_run = np.array([])
        travel_distance_each_run = np.array([])

        # v_in_front (order -1 if no vehicle is in front)
        v_in_front = Vehicle(-1, 0, 0, 0, 0, dummy_vehicle, 1) # dummy vehicle

        for v in self.vehicles:
            speeds_each_run = np.append(speeds_each_run, v.update_speed(tick))
            travel_distance_each_run = np.append(travel_distance_each_run, v.update_travel_distance())
            positions_each_run = np.append(positions_each_run, v.update_position(self.vehicles[0].get_travel_distance()))
            distances_each_run = np.append(distances_each_run, v.update_distance(v_in_front.get_travel_distance()))
            reference_each_run = np.append(reference_each_run, v.get_min_distance())
            v.update_min_distance()
            v_in_front = v

        self.speeds.append(speeds_each_run)
        self.travel_distance.append(travel_distance_each_run)
        self.distances.append(distances_each_run)
        self.positions.append(positions_each_run)
        self.references.append(reference_each_run)

        return speeds_each_run, positions_each_run, distances_each_run

    def get_speeds(self):
        return self.speeds

    def get_travel_distance(self):
        return self.travel_distance

    def get_distances(self):
        return self.distances

    def get_positions(self):
        return self.positions

    def get_references(self):
        return self.references
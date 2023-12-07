from src.platoon.platoon import Platoon
from src.vehicle.vehicle import Vehicle
from src.vehicle.vehicle_specs import dummy_vehicle
import numpy as np

class PlatoonBidirectional(Platoon):

    def __init__(self, num_vehicles, vehicle_specs):
        Platoon.__init__(self, num_vehicles, vehicle_specs)

    def run(self, tick):
        speeds_each_run = np.array([])
        positions_each_run = np.array([])
        distances_each_run = np.array([])
        travel_distance_each_run = np.array([])

        # v_in_front (order -1 if no vehicle is in front)
        v_in_front = Vehicle(-1, dummy_vehicle) # dummy vehicle

        for v in self.vehicles:
            speeds_each_run = np.append(speeds_each_run, v.update_speed(tick, self.vehicles[0].get_current_speed(), v_in_front.get_position()))
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


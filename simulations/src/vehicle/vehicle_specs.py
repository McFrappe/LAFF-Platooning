import numpy as np


class StateSpaceVehicleParameters:
    def __init__(self, h_p, r, m, a):
        self.h_p = h_p
        self.r = r
        self.m = m
        self.a = a


class VehicleSpecs:
    def __init__(self, max_speed, max_acceleration, max_deceleration, mass, state_space_vehicle_parameters):
        self.max_speed_in_km_per_h = max_speed
        self.max_acceleration_in_km_per_h_per_tick = max_acceleration
        self.max_deceleration_in_km_per_h_per_tick = max_deceleration
        self.mass = mass 
        self.state_space_vehicle_parameters = state_space_vehicle_parameters

    def get_max_speed_in_km_per_h(self):
        return self.max_speed_in_km_per_h

    def get_max_acceleration_in_km_per_h_per_tick(self):
        return self.max_acceleration_in_km_per_h_per_tick

    def get_max_deceleration_in_km_per_h_per_tick(self):
        return self.max_deceleration_in_km_per_h_per_tick

    def get_mass_in_kg(self):
        return self.mass 

    def get_state_space_vehicle_parameters(self):
        return self.state_space_vehicle_parameters


truck_h_p = 0.5
truck_r = np.array([200000,200000,200000]) # [r_{i-1}, r_{i}, r_{i+1}] 
truck_m = np.array([6000,6000,6000]) # [m_{i-1}, m_{i}, m_{i+1}] 
truck_a = np.array([0,300000,50000]) # [a_{i-1} (not used), a_{i}, a_{i+1}] 
truck_ss_par = StateSpaceVehicleParameters(truck_h_p, truck_r, truck_m, truck_a)

rc_vehicle_h_p = 0.5
rc_vehicle_r = np.array([200,200,200]) # [r_{i-1}, r_{i}, r_{i+1}] 
rc_vehicle_m = np.array([1.8,1.8,1.8]) # [m_{i-1}, m_{i}, m_{i+1}] 
rc_vehicle_a = np.array([0,300,50]) # [a_{i-1} (not used), a_{i}, a_{i+1}] 
rc_vehicle_ss_par = StateSpaceVehicleParameters(rc_vehicle_h_p, rc_vehicle_r, rc_vehicle_m, rc_vehicle_a)

truck = VehicleSpecs(100, 0.05, 0.2, 6000, truck_ss_par) # TODO: change to 30 000
kit_car = VehicleSpecs(4, 0.04, 0.15, 1, truck_ss_par)
rc_car = VehicleSpecs(30, 0.08, 0.3, 1.8, rc_vehicle_ss_par)
dummy_vehicle = VehicleSpecs(0, 0, 0, 0, truck_ss_par)

import numpy as np
from scipy import signal
from src.vehicle.vehicle import Vehicle
from src.common.constants import tick_in_s
from src.common.state_space import StateSpaceDiscrete


class VehicleBidirectionalStateSpace(Vehicle):
    def __init__(self, order, num_followers, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs, state_space_vehicle_parameters, period):
        Vehicle.__init__(self, order, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs, period)

        self.delta = 0 
        self.ss_par = state_space_vehicle_parameters#StateSpaceVehicleParameters
        self.mass = self.ss_par.m[1]
        self.speed_increase_per_tick = 0

        max_deceleration = vehicle_specs.get_max_deceleration_in_km_per_h_per_tick()
        max_acceleration = vehicle_specs.get_max_acceleration_in_km_per_h_per_tick()
        min_speed = 0
        max_speed = vehicle_specs.get_max_speed_in_km_per_h()

        self.max_deceleration = max_deceleration
        self.max_acceleration = max_acceleration
        self.min_speed = min_speed
        self.max_speed = max_speed

        a11 = -((1+self.ss_par.h_p)-(-1+self.ss_par.h_p)*(self.order % num_followers)/(self.order))*(self.ss_par.r[1]/self.ss_par.m[1])
        a12 = self.ss_par.a[1]
        a21 = -1/self.ss_par.m[1]
        b12 = (1+self.ss_par.h_p)*(self.ss_par.r[0]/self.ss_par.m[0])
        b13 = -(-1+self.ss_par.h_p)*(self.ss_par.r[2]/self.ss_par.m[2])

        A_continuous = np.array([[a11, a12], [a21, 0]])
        B_continuous = np.array([[-1, b12, b13, 0, self.ss_par.a[2]], [0, 1/self.ss_par.m[0], 0, 0, 0]])
        C_continuous = np.array([[1/self.mass, 0]])
        D_continuous = np.array([[0]])

        self.ss = StateSpaceDiscrete(A_continuous, B_continuous, C_continuous, D_continuous, 
                                    tick_in_s, period, 
                                    -max_deceleration, max_acceleration, min_speed, max_speed)

    def update_speed(self, tick, leader_speed, relative_position_infront, momentum_infront, momentum_behind, delta_infront, delta_behind):
        self.delta = self.calculate_positioning_error(leader_speed, relative_position_infront)
        state_variables =  np.array([self.mass*self.speed, self.delta])
        input_variables =  np.array([leader_speed, momentum_infront, momentum_behind, delta_infront, delta_behind])

        self.speed = self.ss.update(tick, state_variables, input_variables)
        return self.speed

    def update_all_speeds(self, speeds_vector):
        self.speeds.append(speeds_vector)

    def update_all_relative_positions(self, relative_positions_vector):
        self.relative_positions.append(relative_positions_vector)

    def calculate_positioning_error(self, leader_speed, relative_position_infront):
        reference_self = self.calculate_valid_reference(leader_speed, self.order)
        reference_infront = self.calculate_valid_reference(leader_speed, self.order-1) # will be 0 for the first follower as desired
        positioning_error = (reference_self - self.position) - (reference_infront - relative_position_infront)

        return -positioning_error

    def calculate_valid_reference(self, leader_speed, order):
        speed_in_m_per_s = leader_speed/3.6
        margin_in_m = 1
        minimal_distance = speed_in_m_per_s * self.period + margin_in_m

        return minimal_distance * order

    def get_momentum(self):
        return self.speed*self.mass 

    def get_delta(self):
        return self.delta 
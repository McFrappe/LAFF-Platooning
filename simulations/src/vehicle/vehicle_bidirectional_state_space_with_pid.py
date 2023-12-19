import numpy as np
from scipy import signal
from src.vehicle.vehicle_bidirectional_state_space import VehicleBidirectionalStateSpace
from src.common.constants import tick_in_s
from src.common.pid_increase import PidIncrease


class VehicleBidirectionalStateSpaceWithPid(VehicleBidirectionalStateSpace):
    def __init__(self, order, num_followers, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs, state_space_vehicle_parameters, period):
        VehicleBidirectionalStateSpace.__init__(self, order, num_followers, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs, state_space_vehicle_parameters, period)
        h_c = 0.50
        self.y_ratio = np.array([[h_c, 1-h_c]])

        self.min_speed = 0
        self.max_speed = vehicle_specs.get_max_speed_in_km_per_h()
        max_deceleration = vehicle_specs.get_max_deceleration_in_km_per_h_per_tick()
        max_acceleration = vehicle_specs.get_max_acceleration_in_km_per_h_per_tick()

        kp = max_acceleration
        ki = 0.01
        kd = 6

        self.pid = PidIncrease(kp, ki, kd, tick_in_s, -max_deceleration, max_acceleration)

    def update_speed(self, tick, leader_speed, relative_position_infront, momentum_infront, momentum_behind, delta_infront, delta_behind):
        self.delta = self.calculate_positioning_error(leader_speed, relative_position_infront)
        state_variables =  np.array([self.mass*self.speed, self.delta])
        input_variables =  np.array([leader_speed, momentum_infront, momentum_behind, delta_infront, delta_behind])
        y_ss = self.ss.update(tick, state_variables, input_variables)

        error = self.distance - self.min_distance
        increase = self.pid.update(error)
        y_pid = min(max(self.speed + increase, self.min_speed), self.max_speed)

        y = np.array([y_ss, y_pid])
        self.speed = (self.y_ratio @ y)[0]
        return self.speed

    def update_min_distance(self):
        speed_in_m_per_s = self.speed/3.6
        margin_in_m = 1 # TODO: should depend on speed and vehicle?
        self.min_distance = speed_in_m_per_s * self.period + margin_in_m 
        return self.min_distance
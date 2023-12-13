import numpy as np
from scipy import signal
from src.vehicle.vehicle_bidirectional_state_space import VehicleBidirectionalStateSpace
from src.common.constants import tick_in_s
from src.common.pid import PID


class VehicleBidirectionalStateSpace(VehicleBidirectionalStateSpace):
    def __init__(self, order, num_followers, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs, state_space_vehicle_parameters, period):
        VehicleBidirectionalStateSpace.__init__(self, order, num_followers, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs, state_space_vehicle_parameters, period)
        h_c = 0.8
        self.y_ratio = np.array([[h_c, 1-h_c]])

        min_speed = 0
        max_speed = vehicle_specs.get_max_speed_in_km_per_h()
        max_deceleration = vehicle_specs.get_max_deceleration_in_km_per_h_per_tick()
        max_acceleration = vehicle_specs.get_max_acceleration_in_km_per_h_per_tick()

        #kp = max_acceleration
        #ki = 0
        #kd = 1

        kp = 10
        ki = 6
        kd = 2

        self.pid = PID(kp, ki, kd, tick_in_s, min_speed, max_speed, max_deceleration, max_acceleration)

    def update_speed(self, tick, distance, leader_speed, relative_position_infront, momentum_infront, momentum_behind, delta_infront, delta_behind):
        if int(tick) % int(self.period/tick_in_s) != 0:
            desired_speed = self.speed + self.speed_increase_per_tick
            y_ss = self.calculate_valid_speed(desired_speed)
        else:
            self.delta = self.calculate_positioning_error(leader_speed, relative_position_infront)
            state_variables = np.array([self.mass*self.speed, self.delta])
            input_variables = np.array([leader_speed, momentum_infront, momentum_behind, delta_infront, delta_behind])

            next_step = self.A_discrete @ state_variables + self.B_discrete @ input_variables
            desired_speed = self.C_discrete @ next_step

            prev_speed = self.speed
            speed_increase = desired_speed[0]-prev_speed
            ticks_per_period = self.period/tick_in_s
            desired_speed_increase_per_tick = speed_increase/ticks_per_period
            y_ss = self.calculate_valid_speed(self.speed+desired_speed_increase_per_tick)
            self.speed_increase_per_tick = y_ss-prev_speed

        error = self.distance - self.min_distance
        y_pid = self.pid.update(error)

        y = np.array([y_ss, y_pid])
        self.speed = self.y_ratio @ y
        return self.speed


        if int(tick) % int(self.period/tick_in_s) != 0:
            desired_speed = self.speed + self.speed_increase_per_tick
            self.speed = self.calculate_valid_speed(desired_speed)
            return self.speed

        self.delta = self.calculate_positioning_error(leader_speed, relative_position_infront)
        state_variables =  np.array([self.mass*self.speed, self.delta])
        input_variables =  np.array([leader_speed, momentum_infront, momentum_behind, delta_infront, delta_behind])

        next_step = self.A_discrete @ state_variables + self.B_discrete @ input_variables
        desired_speed = self.C_discrete @ next_step

        prev_speed = self.speed
        speed_increase = desired_speed[0]-prev_speed
        ticks_per_period = self.period/tick_in_s
        desired_speed_increase_per_tick = speed_increase/ticks_per_period
        self.speed = self.calculate_valid_speed(self.speed+desired_speed_increase_per_tick)
        self.speed_increase_per_tick = self.speed-prev_speed

        return self.speed
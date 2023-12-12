import numpy as np
from src.vehicle.vehicle import Vehicle
from src.common.constants import tick_in_s

class VehicleBidirectional(Vehicle):
    def __init__(self, order, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs, period):
        Vehicle.__init__(self, order, vehicle_specs, period)
        self.speed = init_speed
        self.position = init_position
        self.distance = init_distance
        self.travel_distance = init_travel_distance
        self.integral_sum = 0
        self.prev_velocity_error = 0

    def update_speed(self, tick, leader_speed, relative_position_infront):
        delta_self = self.calculate_positioning_error(leader_speed, relative_position_infront)
        velocity_deviation = self.calculate_velocity_deviation(leader_speed, delta_self)
        velocity_deviation_fin = -velocity_deviation[0][0] * 0.85 + velocity_deviation[0][1] * 0.15 #0.8 and 0.2

        max_acceleration  = self.vehicle_specs.get_max_acceleration_in_km_per_h_per_tick()
        kp = max_acceleration/0.4 #0.2
        ki = 0.00005  # 0.001
        kd = 1 # 0.1
        self.integral_sum += velocity_deviation_fin *kp
        derivative = (velocity_deviation_fin *kp) - self.prev_velocity_error
        self.prev_velocity_error = velocity_deviation_fin *kp

        fs = kp * velocity_deviation_fin + ki * self.integral_sum + kd * derivative
        desired_speed = self.speed + fs
        self.speed = self.calculate_valid_speed(desired_speed)

        desired_speed = self.speed
        self.speed = self.calculate_valid_speed(desired_speed)

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
        margin_in_m = 2
        minimal_distance = speed_in_m_per_s * tick_in_s *2 + margin_in_m

        return minimal_distance * order

    def calculate_velocity_deviation(self, v_leader, positioning_error):
        m_self = 1
        v_self = self.speed
        a_self = 0.8
        p_self = m_self * v_self
        delta_self = positioning_error
        deviation_vector = np.array([[1/np.sqrt(m_self) * (p_self - m_self*v_leader), np.sqrt(a_self) * delta_self]])

        return np.sqrt(1/2)*deviation_vector

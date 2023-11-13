import numpy as np
from src.vehicle.vehicle import Vehicle
from src.common.constants import tick_in_s

class VehicleBidirectional(Vehicle):

    def __init__(self, order, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs):
        Vehicle.__init__(self, order, vehicle_specs)
        self.speed = init_speed
        self.position = init_position
        self.distance = init_distance
        self.travel_distance = init_travel_distance

    # This should be called each tick
    def update_speed(self, tick, leader_speed, relative_position_infront):
        print("----------------------------------")
        print(f"in front: {relative_position_infront}")
        print(f"self: {self.position}")
        delta_self = self.calculate_positioning_error(leader_speed, relative_position_infront) # TODO:
        print(f"delta: {delta_self}")
        velocity_deviation = self.calculate_velocity_deviation(leader_speed, delta_self)
        print(f"velocity deviation: {velocity_deviation}")

        distance_from_min = (self.distance - self.min_distance)
        integral = sum(self.array_distance_errors) # 1 tick * (sum of all distance_errors) gives area
        derivative = self.error_derivative

        if derivative >= 0: # accelerating
            max_acceleration  = self.vehicle_specs.get_max_acceleration_in_km_per_h_per_tick()
            kp = max_acceleration/15
            #ki = 0.002
            ki = 0.1
            kd = 4
        elif distance_from_min < 0.2: # safety margin break with maximum deceleration
            max_deceleration = self.vehicle_specs.get_max_deceleration_in_km_per_h_per_tick()
            kp = max_deceleration*100
            ki = 0
            kd = 0
        else: # normal deceleration
            max_deceleration = self.vehicle_specs.get_max_deceleration_in_km_per_h_per_tick()
            kp = max_deceleration/15
            ki = 0
            kd = 3

        fs = kp * distance_from_min + ki * integral + kd * derivative
        desired_speed = self.speed + fs
        self.speed = self.calculate_valid_speed(desired_speed)
    


        # Calculate the new tick and update old if needed

        # Bidirectional
        # -(reference + relative_position)_infront + (reference - relavtive_position)_self = delta_self
        # Velocity depends on the relative postions. But is also depends on the vehicles speed infront and behind
        # (velocity_infront - velocity_self*(1-h_p) + delta_self*<some constant to represent what the change in velocity should be>)

        # Leader-proceeder
        # (Positioning error for all)
        # (velocity error for all +- Positioning error) will not work needs to be more then that

        # Eather A PID controller or MPC or statespace can be made
        # deviation from equilibrium: X_p = \sqrt{1/2} col(M^{-1/2}(p(t)-Mv_0), A^{1/2}\Delta)
        #M = np.identity(3) # number of vehicles # moment of inertia
        #A = np.identity(3) # acceleration    
        #Delta = 

        desired_speed = self.speed
        self.speed = self.calculate_valid_speed(desired_speed)

        return self.speed


    def update_all_speeds(self, speeds_vector):
        self.speeds.append(speeds_vector)

    def update_all_relative_positions(self, relative_positions_vector):
        self.relative_positions.append(relative_positions_vector)

    def calculate_positioning_error(self, leader_speed, relative_position_infront):
        """ Positioning error according to bidirectional paper """
         
        reference_self = self.calculate_valid_reference(leader_speed, self.order)
        reference_infront = self.calculate_valid_reference(leader_speed, self.order-1) # will be 0 for the first follower as desired
        positioning_error = (reference_self - self.position) - (reference_infront - relative_position_infront)

        return positioning_error

    def calculate_valid_reference(self, leader_speed, order):
        """ Determine the minimal distance and then multiply it with the order to get the reference """

        speed_in_m_per_s = leader_speed/3.6
        margin_in_m = 2
        minimal_distance = speed_in_m_per_s * tick_in_s *2 + margin_in_m # should be communication instead

        return minimal_distance * order

    def calculate_velocity_deviation(self, v_leader, positioning_error):
        # we dont need to know the hole matrix we only need to know the relevant elements within it.
        m_self = 1  # mass of self
        v_self = self.speed  # velocity of self
        # v_leader = 1  # velocity of leader
        a_self = 1  # positioning coupling of self
        p_self = m_self * v_self
        delta_self = positioning_error  # positioning error
        deviation_vector = np.array([[1/np.sqrt(m_self) * (p_self - m_self*v_leader), a_self * delta_self]])
        return np.sqrt(1/2)*deviation_vector

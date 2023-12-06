import numpy as np
from scipy import signal
from src.vehicle.vehicle import Vehicle
from src.common.constants import tick_in_s

class VehicleBidirectionalStateSpace(Vehicle):

    def __init__(self, order, num_followers, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs):
        Vehicle.__init__(self, order, vehicle_specs)
        self.speed = init_speed
        self.position = init_position
        self.distance = init_distance
        self.travel_distance = init_travel_distance
        self.integral_sum = 0
        self.prev_velocity_error = 0
        self.delta = 0 

        # continues time
        #r = np.array([10000,10000,5000]) # [r_{i-1}, r_{i}, r_{i+1}] 
        #m = np.array([6000,6000,6000]) # [m_{i-1}, m_{i}, m_{i+1}] 
        #a = np.array([0,50000,20000]) # [a_{i-1} (not used), a_{i}, a_{i+1}] 
        h_p = 0.5

        # discrete time
        r = np.array([200000,200000,200000]) # [r_{i-1}, r_{i}, r_{i+1}] 
        #r = np.array([100000,100000,100000]) # [r_{i-1}, r_{i}, r_{i+1}] 
        m = np.array([6000,6000,6000]) # [m_{i-1}, m_{i}, m_{i+1}] 
        #m = np.array([6000,6000,6000]) # [m_{i-1}, m_{i}, m_{i+1}] 
        #a = np.array([0,300000,50000]) # [a_{i-1} (not used), a_{i}, a_{i+1}] 
        a = np.array([0,300000,50000]) # [a_{i-1} (not used), a_{i}, a_{i+1}] 

        self.mass = m[1]

        self.A_continuous = np.array([[-((1+h_p)-(-1+h_p)*(self.order % num_followers)/(self.order))*(r[1]/m[1]), a[1]], [-1/m[1], 0]])
        self.B_continuous = np.array([[-1, (1+h_p)*(r[0]/m[0]), -(-1+h_p)*(r[2]/m[2]), 0, a[2]], [0, 1/m[0], 0, 0, 0]])
        self.C_continuous = np.array([[1/self.mass, 0]])
        self.D_continuous = np.array([[0]])

        # Convert to discrete-time system
        self.A_discrete, self.B_discrete, self.C_discrete, self.D_discrete, _ = signal.cont2discrete(
            (self.A_continuous, self.B_continuous, self.C_continuous, self.D_continuous),
            dt=tick_in_s,
            method='zoh'  # You can choose other discretization methods
        )

    def update_speed(self, tick, leader_speed, relative_position_infront, momentum_infront, momentum_behind, delta_infront, delta_behind):
        
        self.delta = self.calculate_positioning_error(leader_speed, relative_position_infront)
        state_variables =  np.array([self.mass*self.speed, self.delta])
        input_variables =  np.array([leader_speed, momentum_infront, momentum_behind, delta_infront, delta_behind])

        next_step = self.A_discrete @ state_variables + self.B_discrete @ input_variables
        desired_speed = self.C_discrete @ next_step
        self.speed = self.calculate_valid_speed(desired_speed[0]) # we want a scaler not a np array with single value

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
        margin_in_m = 0.5
        minimal_distance = speed_in_m_per_s * tick_in_s + margin_in_m

        return minimal_distance * order

    def get_momentum(self):
        return self.speed*self.mass 

    def get_delta(self):
        return self.delta 
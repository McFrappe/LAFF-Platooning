from scipy import signal
from src.common.constants import tick_in_s
import numpy as np


class StateSpace():
    def __init__(self, A_continuous, B_continuous, C_continuous, D_continuous, period):
        self.period = period
        self.ticks_per_period = period/tick_in_s
        self.speed_increase_per_tick = 0

        # Convert to discrete-time system
        self.A_discrete, self.B_discrete, self.C_discrete, self.D_discrete, _ = signal.cont2discrete(
            (A_continuous, B_continuous, C_continuous, D_continuous),
            dt=period,
            method='zoh'  # You can choose other discretization methods
        )

    def update(self, tick, state_variables, input_variables):
        if int(tick) % int(self.period/tick_in_s) != 0:
            desired_speed = self.speed + self.speed_increase_per_tick
            speed = self.calculate_valid_speed(desired_speed)
            return speed

        next_step = self.A_discrete @ state_variables + self.B_discrete @ input_variables
        desired_speed = self.C_discrete @ next_step

        prev_speed = self.speed
        speed_increase = desired_speed[0]-prev_speed
        desired_speed_increase_per_tick = speed_increase/self.ticks_per_period
        speed = self.calculate_valid_speed(self.speed+desired_speed_increase_per_tick)
        self.speed_increase_per_tick = self.speed-prev_speed
 
        return 1
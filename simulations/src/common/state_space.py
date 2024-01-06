from scipy import signal
import numpy as np


class StateSpaceDiscrete():
    def __init__(self, A_continuous, B_continuous, C_continuous, D_continuous, update_period: float, ss_period: float, min_increase: float, max_increase: float, min_output: float, max_output: float, init_output: float):
        self.update_period = update_period
        self.ticks_per_period = ss_period/update_period
        self.increase_per_tick = 0
        self.output = init_output 
        self.min_increase = min_increase
        self.max_increase = max_increase
        self.min_output = min_output
        self.max_output = max_output

        # Convert to discrete-time system
        self.A_discrete, self.B_discrete, self.C_discrete, self.D_discrete, _ = signal.cont2discrete(
            (A_continuous, B_continuous, C_continuous, D_continuous),
            dt=ss_period,
            method='zoh'  # You can choose other discretization methods
        )

    def update(self, tick, state_variables, input_variables):
        ticks_in_ms = int(tick*1000)
        ticks_per_period_in_ms = int(self.ticks_per_period*1000)

        if ticks_in_ms % ticks_per_period_in_ms != 0:
            desired_output = self.output + self.increase_per_tick
            self.output = min(max(desired_output, self.min_output), self.max_output)
            return self.output

        next_step = self.A_discrete @ state_variables + self.B_discrete @ input_variables
        desired_output = self.C_discrete @ next_step

        desired_increase = desired_output[0] - self.output
        desired_increase_per_tick = desired_increase/self.ticks_per_period
        self.increase_per_tick = min(max(desired_increase_per_tick, self.min_increase), self.max_increase)

        desired_output = self.output + self.increase_per_tick
        self.output = min(max(desired_output, self.min_output), self.max_output)
 
        return self.output

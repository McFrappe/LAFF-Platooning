from scipy import signal
from gekko import GEKKO
import numpy as np

class MPCStateSpaceDiscrete():
    def __init__(self, A_continuous, B_continuous, C_continuous, D_continuous, update_period: float, ss_period: float, min_increase: float, max_increase: float, min_output: float, max_output: float):
        self.update_period = update_period
        self.ticks_per_period = ss_period/update_period
        self.increase_per_tick = 0
        self.output = 0 
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

        self.m = GEKKO()
        x,y,u = self.m.state_space(self.A_discrete,self.B_discrete,self.C_discrete,D=None)

        # MVs
        v0 = u[0] # feedforward
        p_f = u[1] # feedforward
        p_b = u[2] # MV
        d_f = u[3] # MV
        d_b = u[4] # MV
        
        # CVs
        p_i = y[0]
        #d_i = y[1]
        
        # Parameters
        mass = 6000
        self.m.options.imode = 6
        self.m.options.nodes = 5
        
        # Feedforward
        u[0].lower = 0 
        u[0].upper = 60
        #u[0].dcost = 1
        u[0].status = 1
        
        u[1].lower = 0
        u[1].upper = 60*mass
        #u[1].dcost = 1
        u[1].status = 1
        
        # MVs
        u[2].lower = 0
        u[2].upper = 60*mass
        u[2].dcost = 1
        u[2].status = 1
        
        u[3].lower = -0.2
        u[3].upper = 3
        #u[3].dcost = 1
        u[3].status = 1
        
        u[4].lower = -0.2
        u[4].upper = 3
        #u[4].dcost = 1
        u[4].status = 1

        # CV tuning
        y[0].tau = 60 # first order time constant for trajectories
        y[0].tr_init = 0 # dead band
        y[0].sphi= 0 # SPHI = upper set point
        y[0].splo= 100 # SPLO = lower set point
        y[0].status = 1

        self.m.options.imode = 6 # CTL
        self.m.options.nodes = 5 # Higher better accuracy but longer time

    def update(self, tick, state_variables, input_variables):
        self.m.time = np.linspace(0,10,10) #[0, 0.1, 0.2, 0.4, 1, 1.5, 2, 3, 4]

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

from src.vehicle.vehicle import Vehicle
from src.common.constants import tick_in_s
from src.common.pid import PID
from src.common.pid_increase import PidIncrease


class VehiclePidIncreaseDistance(Vehicle):
    def __init__(self, order, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs, period):
        Vehicle.__init__(self, order, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs, period)

        min_speed = 0
        max_speed = vehicle_specs.get_max_speed_in_km_per_h()
        max_deceleration = vehicle_specs.get_max_deceleration_in_km_per_h_per_tick()
        max_acceleration = vehicle_specs.get_max_acceleration_in_km_per_h_per_tick()

        kp = 1
        ki = 0.02
        kd = 8

        self.pid = PidIncrease(kp, ki, kd, tick_in_s, -max_deceleration, max_acceleration)

    # This should be called each tick
    def update_speed(self, tick):
        error = self.distance - self.min_distance
        self.speed = self.speed + self.pid.update(error)
        return self.speed



        distance_from_min = (self.distance - self.min_distance)
        integral = self.error_integral
        derivative = self.error_derivative

        if True: #derivative >= 0: # accelerating
            max_acceleration  = self.vehicle_specs.get_max_acceleration_in_km_per_h_per_tick()
            #kp = max_acceleration/15
            kp = 1#max_acceleration/15
            #ki = 0.002
            ki = 0#.1
            kd = 16
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
    
        return self.speed

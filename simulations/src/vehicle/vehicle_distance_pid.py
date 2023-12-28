from src.vehicle.vehicle import Vehicle
from src.common.constants import tick_in_s
from src.common.pid import PID
from src.common.pid_increase import PidIncrease


class VehiclePidDistance(Vehicle):
    def __init__(self, order, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs, period):
        Vehicle.__init__(self, order, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs, period)

        min_speed = 0
        max_speed = vehicle_specs.get_max_speed_in_km_per_h()
        max_deceleration = vehicle_specs.get_max_deceleration_in_km_per_h_per_tick()
        max_acceleration = vehicle_specs.get_max_acceleration_in_km_per_h_per_tick()

        kp = 95
        ki = 8 
        kd = 40

        #kp = 140#4
        #ki = kp/20 #3
        #kd = kp*30



        self.pid = PID(kp, ki, kd, tick_in_s, min_speed, max_speed, -max_deceleration, max_acceleration)

    # This should be called each tick
    def update_speed(self, tick):
        error = self.distance - self.min_distance
        self.speed = self.pid.update(error)

        return self.speed

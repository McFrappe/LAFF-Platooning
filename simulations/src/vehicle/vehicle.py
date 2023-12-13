from src.common.constants import tick_in_s

class Vehicle:

    t_c = 10  # number of ticks for period of communication (10ms*t_c = 100ms)
    array_distance_errors_len = 100

    def __init__(self, order, vehicle_specs, period) -> None:
        self.vehicle_specs = vehicle_specs
        self.speed = 0  # initially it the vehicle stands still
        self.speed_old = 0  # updates each period t_c
        self.position = 0  # the distance to the leader
        self.travel_distance = 0 # current distance traveled of the vehicle in meters
        self.distance = 0  # distance to the vehicle in front (-1 if no one is in front)
        self.order = order  # number of vehicles in front (first has order = 0)
        self.array_distance_errors = [0]*self.array_distance_errors_len
        self.array_distance_errors_pointer = 0
        self.error_derivative = 0
        self.error_integral = 0
        self.prev_distance_error = 0
        self.min_distance = 9  # how close the vehicles should be to each other, depends on speed. (m)
        self.period = period

    # This should be called each tick
    def update_travel_distance(self):
        speed_in_m_per_s = self.speed/3.6
        distance_traveled_per_tick_in_m = speed_in_m_per_s * tick_in_s
        self.travel_distance = self.travel_distance + distance_traveled_per_tick_in_m

        return self.travel_distance

    # This should be called each tick
    def update_position(self, travel_distance_leader):
        self.position = travel_distance_leader - self.travel_distance

        return self.position

    # This should be called each tick
    def update_distance(self, travel_distance_of_vehicle_in_front):
        is_leader = self.order == 0

        if is_leader:
            self.distance = 0
        else:
            self.distance = travel_distance_of_vehicle_in_front - self.travel_distance

        # TODO: remove only calculate the distance.
        error = self.distance - self.min_distance
        self.error_derivative = (error - self.prev_distance_error)/tick_in_s
        self.error_integral += error*tick_in_s
        self.prev_distance_error = error

        self.array_distance_errors[self.array_distance_errors_pointer] = error
        if self.array_distance_errors_pointer == self.array_distance_errors_len-1:
            self.array_distance_errors_pointer = 0

        return self.distance

    def update_min_distance(self):
        speed_in_m_per_s = self.speed/3.6
        margin_in_m = 0.2 # TODO: should depend on speed and vehicle?
        self.min_distance = speed_in_m_per_s * tick_in_s *2 + margin_in_m 
        return self.min_distance

    def calculate_valid_speed(self, desired_speed):
        # Calculate valid speed, i.e., between 0 and 100
        max_speed = self.vehicle_specs.get_max_speed_in_km_per_h()
        allowed_speed = min(max(desired_speed, 0), max_speed)

        # Calculate valid acceleration/deceleration
        speed_up  = allowed_speed - self.speed > 0
        if speed_up:
            acceleration = allowed_speed - self.speed
            max_acceleration = self.vehicle_specs.get_max_acceleration_in_km_per_h_per_tick()
            speed = self.speed + min(acceleration, max_acceleration)
        else:
            deceleration = self.speed - allowed_speed
            max_deceleration = self.vehicle_specs.get_max_deceleration_in_km_per_h_per_tick()
            speed = self.speed - min(deceleration, max_deceleration)

        return speed

    def get_current_speed(self):
        return self.speed

    def get_old_speed(self):
        return self.speed_old

    def get_order(self):
        return self.order

    def get_position(self):
        return self.position

    def get_travel_distance(self):
        return self.travel_distance
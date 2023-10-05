class Vehicle:

    t_c = 10  # number of steps for period of communication (10ms*t_c = 100ms)
    max_speed = 100 # (km/h)
    max_acceleration = 0.05 # speed (km/h) / step
    max_deceleration = 0.2 # speed (km/h) / step
    array_distance_errors_len = 100

    def __init__(self, order) -> None:
        self.speed = 0  # initially it the vehicle stands still
        self.speed_old = 0  # updates each period t_c
        self.position = 0  # current position of the vehicle in meters
        self.distance = 0  # distance to the vehicle in front (-1 if no one is in front)
        self.order = order  # number of vehicles in front (first has order = 0)
        self.array_distance_errors = [0]*self.array_distance_errors_len
        self.array_distance_errors_pointer = 0
        self.error_derivative = 0
        self.prev_distance_error = 0
        self.min_distance = 9  # how close the vehicles should be to each other, depends on speed. (m)

    # This should be called each step
    def update_speed(self, step):
        if self.speed < self.max_speed:
            self.speed = self.speed + self.max_acceleration/2

        return self.speed

    # This should be called each step
    def update_position(self):
        speed_ms = self.speed/3.6
        distance_traveled_per_step_m = speed_ms/100  # TODO: should work with any step size
        self.position = self.position + distance_traveled_per_step_m

        return self.position

    # This should be called each step
    def update_distance(self, position_of_vehicle_in_front):
        is_leader = self.order == 0

        if is_leader:
            self.distance = 0
        else:
            self.distance = position_of_vehicle_in_front - self.position

        error = self.distance - self.min_distance
        self.error_derivative = error - self.prev_distance_error # ms
        self.prev_distance_error = error

        self.array_distance_errors[self.array_distance_errors_pointer] = error
        #self.array_distance_errors_pointer = 1 + self.array_distance_errors_pointer
        if self.array_distance_errors_pointer == self.array_distance_errors_len-1:
            self.array_distance_errors_pointer = 0

        #self.sum_distance_errors = self.sum_distance_errors + error
        return self.distance

    def calculate_valid_speed(self, desired_speed):
        # Calculate valid speed, i.e., between 0 and 100
        allowed_speed = min(max(desired_speed, 0), 100)

        # Calculate valid acceleration/deceleration
        speed_up  = allowed_speed - self.speed > 0
        if speed_up:
            speed = self.speed + min(allowed_speed - self.speed, self.max_acceleration)
        else:
            speed = self.speed - min(self.speed - allowed_speed, self.max_deceleration)

        return speed

    def get_current_speed(self):
        return self.speed

    def get_old_speed(self):
        return self.speed_old

    def get_order(self):
        return self.order

    def get_position(self):
        return self.position
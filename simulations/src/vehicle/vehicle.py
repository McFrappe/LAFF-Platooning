class Vehicle:

    t_c = 10  # number of steps for period of communication (10ms*t_c = 100ms)
    max_speed = 100 # (km/h)
    max_acceleration = 0.05 # speed (km/h) / step
    max_deceleration = 0.2 # speed (km/h) / step

    def __init__(self, order, inti_speed=0, inti_position=0, inti_distance=0) -> None:
        self.speed = inti_speed  # initially it the vehicle stands still
        self.speed_old = 0  # updates each period t_c
        self.position = inti_position  # current position of the vehicle in meters
        self.distance = inti_distance  # distance to the vehicle in front (-1 if no one is in front)
        self.order = order  # number of vehicles in front (first has order = 0)
        self.sum_distance_errors = 0
        self.min_distance = 9  # how close the vehicles should be to each other, depends on speed. (m)

    # This should be called each step
    def update_speed(self, step):
        # Calculate the new step and update old if needed
        is_leader = self.order == 0

        if is_leader:
            #if self.speed < self.max_speed:
            #    self.speed = self.speed + self.max_acceleration/2
            #elif step > 60: # TODO: should change to when it actually should start slowing down
            #    self.speed = self.speed - self.max_acceleration/2
            # else remain constant speed
            pass
        else: 
            distance_from_min = (self.distance - self.min_distance)
            kp = 0.5
            ki = 0
            integral = 10 * self.sum_distance_errors # 10 ms for each step * (sum of all distance_errors) gives area
            fs = kp * distance_from_min + ki * integral
            desired_speed = self.speed + fs
            self.speed = self.calculate_valid_speed(desired_speed)

        return self.speed

    # This should be called each step
    def update_position(self):
        speed_ms = self.speed/3.6
        distance_traveled_per_step_m = speed_ms/100
        self.position = self.position + distance_traveled_per_step_m

        return self.position

    # This should be called each step
    def update_distance(self, position_of_vehicle_in_front):
        is_leader = self.order == 0

        if is_leader:
            self.distance = 0
        else:
            self.distance = position_of_vehicle_in_front - self.position
        
        self.sum_distance_errors = self.sum_distance_errors + (self.distance-self.min_distance)
        return self.distance

    def calculate_valid_speed(self, desired_speed):
        # Calculate valid speed, i.e., between 0 and 100
        allowed_speed = min(max(int(desired_speed), 0), 100)

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
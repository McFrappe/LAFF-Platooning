class Vehicle:

    # Platoon
    # N: number of vehicles 
    # order: number of vehicles in front (first has order = 0)
    # speed
    # velocity
    # distance: distance to the vehicle in front (-1 if no one is in front)
    # t_c: period in which sent data is updated
    # step: how often speed and distance should update
    # position: starts with 0 and increases in next step if speed > 0
    # all update methods should be called each step (iteration of the loop)

    t_c = 10  # number of steps for period of communication
    min_distance = 9  # number of steps for period of communication
    max_speed = 100
    max_acceleration = 4 # speed / step (same as deceleration)
    max_deceleration = 5 # speed / step

    def __init__(self, order) -> None:
        self.speed = 0  # initially it the vehicle stands still
        self.speed_old = 0  # updates each period t_c
        self.position = 0   
        self.distance = 0
        self.order = order

    def set_order(self, order) -> None:
        self.order = order

    # This should be called each step
    def update_speed(self, step):
        # Calculate the new step and update old if needed
        is_leader = self.order == 0

        # TODO: what to do with float as speed
        if is_leader:
            if self.speed < self.max_speed:
                self.speed = self.speed+1
            elif step > 60: # TODO: should change to when it actually should start slowing down
                self.speed = self.speed-1
            # else remain constant speed
        else: 
            distance_from_min = (self.distance - self.min_distance)
            desired_speed = self.speed + (self.speed/10 + distance_from_min)/2  # some margin so we aim to be just close to the distance 
            self.speed = self.calculate_valid_speed(desired_speed)

        return self.speed

    # This should be called each step
    def update_position(self):
        self.position = self.position + self.speed
        return self.position

    # This should be called each step
    def update_distance(self, position_of_vehicle_in_front):
        is_leader = self.order == 0

        if is_leader:
            self.distance = 0
        else:
            self.distance = position_of_vehicle_in_front - self.position

        return self.distance

    def calculate_valid_speed(self, desired_speed):
        # Calculate valid top speed
        allowed_speed = min(max(int(desired_speed), 0), 100)

        # Calculate valid acceleration
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
from src.vehicle.vehicle import Vehicle

class VehicleBidirectional(Vehicle):

    def __init__(self, order, init_speed, init_position, init_distance, vehicle_specs):
        Vehicle.__init__(self, order, vehicle_specs)
        # TODO: save the position for all of the vehicles 
        # could be a global object or each save there data.
        # if each has there own data you could introduce disturbance 
        # which becomes more realistic. We can then have a error rate of 

        self.speeds = [] #np array  S \times N, where S is number of ticks
        self.relative_positions = [] #np array  S \times N, where S is number of ticks

    # This should be called each ticks
    def update_speed(self, tick):
        # Calculate the new tick and update old if needed
        is_leader = self.order == 0

        if is_leader:
            if self.speed < self.max_speed:
                self.speed = self.speed + self.max_acceleration/2
            elif tick > 60: # TODO: should change to when it actually should start slowing down
                self.speed = self.speed - self.max_acceleration/2
            # else remain constant speed
            pass
        else: 
            desired_speed = self.speed
            self.speed = self.calculate_valid_speed(desired_speed)

        return self.speed


    def update_all_speeds(self, speeds_vector):
        self.speeds.append(speeds_vector)

    def update_all_relative_positions(self, relative_positions_vector):
        self.relative_positions.append(relative_positions_vector)
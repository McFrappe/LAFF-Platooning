from src.vehicle.vehicle import Vehicle

class VehiclePidDistance(Vehicle):

    def __init__(self, order, init_speed, init_position, init_distance):
        Vehicle.__init__(self, order)
        self.speed = init_speed
        self.position = init_position
        self.distance = init_distance

    # This should be called each step
    def update_speed(self, step):
        # Calculate the new step and update old if needed
        is_leader = self.order == 0

        if is_leader:
            if self.speed < self.max_speed/2:
                self.speed = self.speed + self.max_acceleration/2
            elif step > 60: # TODO: should change to when it actually should start slowing down
                self.speed = self.speed - self.max_acceleration/2
            # else remain constant speed
            pass
        else: 
            distance_from_min = (self.distance - self.min_distance)
            kp = 0.05/17
            ki = 0.01
            kd = 3
            integral = sum(self.array_distance_errors) # 1 step * (sum of all distance_errors) gives area
            derivative = self.error_derivative
            print(self.order, distance_from_min, derivative)
            fs = kp * distance_from_min + ki * integral + kd * derivative
            desired_speed = self.speed + fs
            print(self.order, desired_speed)
            self.speed = self.calculate_valid_speed(desired_speed)
            print(self.order, self.speed)

        return self.speed

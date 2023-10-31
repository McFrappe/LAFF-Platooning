from src.vehicle.vehicle import Vehicle

class VehiclePidDistance(Vehicle):

    def __init__(self, order, init_speed, init_position, init_distance, vehicle_specs):
        Vehicle.__init__(self, order, vehicle_specs)
        self.speed = init_speed
        self.position = init_position
        self.distance = init_distance

        # senario 3
        self.leader_accelerating = True
        self.leader_initial_accelerating = True

    # This should be called each tick
    def update_speed(self, tick):
        distance_from_min = (self.distance - self.min_distance)
        kp = 0.05/17
        ki = 0.01
        kd = 3
        integral = sum(self.array_distance_errors) # 1 tick * (sum of all distance_errors) gives area
        derivative = self.error_derivative

        fs = kp * distance_from_min + ki * integral + kd * derivative
        desired_speed = self.speed + fs
        self.speed = self.calculate_valid_speed(desired_speed)
    
        # TODO: Seperate PID controller for deceleration
        #if tick >= 1:
        #    distance_from_min = (self.distance - self.min_distance)
        #    max_deceleration = self.vehicle_specs.get_max_deceleration_in_km_per_h_per_tick()
        #    kp = max_deceleration/2.17 # shoulde be an adequeate distance
        #    ki = 0.01
        #    kd = 3
        #    integral = sum(self.array_distance_errors) # 1 tick * (sum of all distance_errors) gives area
        #    derivative = self.error_derivative

        #    fs = kp * distance_from_min + ki * integral + kd * derivative
        #    desired_speed = self.speed + fs
    
        #    desired_speed = self.speed - max_deceleration
        #    self.speed = self.calculate_valid_speed(desired_speed)

        return self.speed

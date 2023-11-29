from src.vehicle.vehicle import Vehicle

class VehiclePidDistance(Vehicle):

    def __init__(self, order, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs):
        Vehicle.__init__(self, order, vehicle_specs)
        self.speed = init_speed
        self.position = init_position
        self.distance = init_distance
        self.travel_distance = init_travel_distance


    # This should be called each tick
    def update_speed(self, tick):
        distance_from_min = (self.distance - self.min_distance)
        integral = sum(self.array_distance_errors) # 1 tick * (sum of all distance_errors) gives area
        derivative = self.error_derivative

        if True: #derivative >= 0: # accelerating
            max_acceleration  = self.vehicle_specs.get_max_acceleration_in_km_per_h_per_tick()
            kp = max_acceleration/15
            #ki = 0.002
            ki = 0#.1
            kd = 4
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

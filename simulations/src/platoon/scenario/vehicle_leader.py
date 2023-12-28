from src.vehicle.vehicle import Vehicle


class VehicleLeaderS1(Vehicle):
    def __init__(self, order, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs, period):
        Vehicle.__init__(self, order, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs, period)

        self.leader_accelerating = True
        self.leader_initial_accelerating = True


    def base_update_speed(self, tick):
        desired_speed = self.speed

        max_speed = self.vehicle_specs.get_max_speed_in_km_per_h()
        if self.speed < max_speed/2 and tick < 6000: #self.max_speed/2:
            max_acceleration = self.vehicle_specs.get_max_acceleration_in_km_per_h_per_tick()
            desired_speed = self.speed + max_acceleration/2
        elif tick > 6000: # TODO: should change to when it actually should start slowing down
            max_deceleration = self.vehicle_specs.get_max_deceleration_in_km_per_h_per_tick()
            desired_speed = self.speed - max_deceleration/24

        self.speed = self.calculate_valid_speed(desired_speed)

        return self.speed


class VehicleLeaderS2(Vehicle):
    def __init__(self, order, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs, period):
        Vehicle.__init__(self, order, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs, period)

        self.leader_accelerating = True
        self.leader_initial_accelerating = True


    def base_update_speed(self, tick):
        max_deceleration = self.vehicle_specs.get_max_deceleration_in_km_per_h_per_tick()
        desired_speed = self.speed - max_deceleration
        self.speed = self.calculate_valid_speed(desired_speed)

        return self.speed


class VehicleLeaderS3(Vehicle):
    def __init__(self, order, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs, period):
        Vehicle.__init__(self, order, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs, period)

        self.leader_accelerating = True
        self.leader_initial_accelerating = True


    def base_update_speed(self, tick):
        max_speed = self.vehicle_specs.get_max_speed_in_km_per_h()
        if int(self.speed) == int(max_speed/(5/3)):
            self.leader_accelerating = False
            self.leader_initial_accelerating = False
        elif int(self.speed) == int(max_speed/2.5) and not self.leader_initial_accelerating: # may fail?
            self.leader_accelerating = True
        
        max_acceleration = self.vehicle_specs.get_max_acceleration_in_km_per_h_per_tick()
        max_deceleration = self.vehicle_specs.get_max_deceleration_in_km_per_h_per_tick()
        if self.leader_accelerating:
            desired_speed = self.speed + max_acceleration/8
        else: 
            desired_speed = self.speed - max_deceleration/30
        self.speed = self.calculate_valid_speed(desired_speed)

        return self.speed


class VehicleLeaderS4(Vehicle):
    def __init__(self, order, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs, period):
        Vehicle.__init__(self, order, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs, period)

        self.leader_accelerating = True
        self.leader_initial_accelerating = True


    def base_update_speed(self, tick):
        max_speed = self.vehicle_specs.get_max_speed_in_km_per_h()
        if int(self.speed) == int(max_speed/1.25):
            self.leader_accelerating = False
            self.leader_initial_accelerating = False
        elif int(self.speed) == int(max_speed/2.5) and not self.leader_initial_accelerating: # may fail?
            self.leader_accelerating = True
        
        max_acceleration = self.vehicle_specs.get_max_acceleration_in_km_per_h_per_tick()
        max_deceleration = self.vehicle_specs.get_max_deceleration_in_km_per_h_per_tick()

        if self.leader_accelerating:
            desired_speed = self.speed + max_acceleration/2
        else: 
            desired_speed = self.speed - max_deceleration/2
        self.speed = self.calculate_valid_speed(desired_speed)

        return self.speed


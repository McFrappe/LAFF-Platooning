class VehicleSpecs:
    def __init__(self, max_speed, max_acceleration, max_deceleration):
        self.max_speed_in_km_per_h = max_speed
        self.max_acceleration_in_km_per_h_per_step = max_acceleration
        self.max_deceleration_in_km_per_h_per_step = max_deceleration

    def get_max_speed_in_km_per_h(self):
        return self.max_speed_in_km_per_h

    def get_max_acceleration_in_km_per_h_per_step(self):
        return self.max_acceleration_in_km_per_h_per_step

    def get_max_deceleration_in_km_per_h_per_step(self):
        return self.max_deceleration_in_km_per_h_per_step


truck = VehicleSpecs(55, 0.05, 0.2)  # TODO: should change too 100 km/h
kit_car = VehicleSpecs(4, 0.04, 0.15)
rc_car = VehicleSpecs(30, 0.08, 0.3)
dummy_vehicle = VehicleSpecs(0, 0, 0)

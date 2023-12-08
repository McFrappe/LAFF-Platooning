class VehicleSpecs:
    def __init__(self, max_speed, max_acceleration, max_deceleration, mass):
        self.max_speed_in_km_per_h = max_speed
        self.max_acceleration_in_km_per_h_per_tick = max_acceleration
        self.max_deceleration_in_km_per_h_per_tick = max_deceleration
        self.mass = mass 

    def get_max_speed_in_km_per_h(self):
        return self.max_speed_in_km_per_h

    def get_max_acceleration_in_km_per_h_per_tick(self):
        return self.max_acceleration_in_km_per_h_per_tick

    def get_max_deceleration_in_km_per_h_per_tick(self):
        return self.max_deceleration_in_km_per_h_per_tick

    def get_mass_in_kg(self):
        return self.mass 


truck = VehicleSpecs(100, 0.05, 0.2, 6000) # TODO: change to 30 000
kit_car = VehicleSpecs(4, 0.04, 0.15, 2)
rc_car = VehicleSpecs(30, 0.08, 0.3, 4)
dummy_vehicle = VehicleSpecs(0, 0, 0, 0)

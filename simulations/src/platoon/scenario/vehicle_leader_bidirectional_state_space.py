from src.platoon.scenario.vehicle_leader import VehicleLeaderS1, VehicleLeaderS2, VehicleLeaderS3, VehicleLeaderS4


class VehicleLeaderBidirectionalStateSpaceS1(VehicleLeaderS1):
    def __init__(self, order, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs):
        VehicleLeaderS1.__init__(self, order, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs)
        self.mass = vehicle_specs.get_mass_in_kg()

    def update_speed(self, tick, leader_speed, relative_position_infront, momentum_infront, momentum_behind, delta_infront, delta_behind):
        return super().base_update_speed(tick)

    def get_momentum(self):
        return self.speed * self.mass

    def get_delta(self):
        return 0 # should never be called


class VehicleLeaderBidirectionalStateSpaceS2(VehicleLeaderS2):
    def __init__(self, order, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs):
        VehicleLeaderS1.__init__(self, order, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs)
        self.mass = vehicle_specs.get_mass_in_kg()

    def update_speed(self, tick, leader_speed, relative_position_infront, momentum_infront, momentum_behind, delta_infront, delta_behind):
        return super().base_update_speed(tick)

    def get_momentum(self):
        return self.speed * self.mass

    def get_delta(self):
        return 0 # should never be called


class VehicleLeaderBidirectionalStateSpaceS3(VehicleLeaderS3):
    def __init__(self, order, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs):
        VehicleLeaderS3.__init__(self, order, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs)
        self.mass = vehicle_specs.get_mass_in_kg()

    def update_speed(self, tick, leader_speed, relative_position_infront, momentum_infront, momentum_behind, delta_infront, delta_behind):
        return super().base_update_speed(tick)

    def get_momentum(self):
        return self.speed * self.mass

    def get_delta(self):
        return 0 # should never be called


class VehicleLeaderBidirectionalStateSpaceS4(VehicleLeaderS4):
    def __init__(self, order, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs):
        VehicleLeaderS1.__init__(self, order, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs)
        self.mass = vehicle_specs.get_mass_in_kg()

    def update_speed(self, tick, leader_speed, relative_position_infront, momentum_infront, momentum_behind, delta_infront, delta_behind):
        return super().base_update_speed(tick)

    def get_momentum(self):
        return self.speed * self.mass

    def get_delta(self):
        return 0 # should never be called

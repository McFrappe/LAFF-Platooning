from src.platoon.scenario.vehicle_leader import VehicleLeaderS1, VehicleLeaderS2, VehicleLeaderS3, VehicleLeaderS4


class VehicleLeaderDistancePidS1(VehicleLeaderS1):
    def __init__(self, order, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs):
        VehicleLeaderS1.__init__(self, order, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs)

    def update_speed(self, tick):
        super().base_update_speed(tick)


class VehicleLeaderDistancePidS2(VehicleLeaderS2):
    def __init__(self, order, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs):
        VehicleLeaderS1.__init__(self, order, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs)

    def update_speed(self, tick):
        super().base_update_speed(tick)


class VehicleLeaderDistancePidS3(VehicleLeaderS3):
    def __init__(self, order, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs):
        VehicleLeaderS3.__init__(self, order, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs)

    def update_speed(self, tick):
        super().base_update_speed(tick)


class VehicleLeaderDistancePidS4(VehicleLeaderS4):
    def __init__(self, order, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs):
        VehicleLeaderS1.__init__(self, order, init_speed, init_travel_distance, init_position, init_distance, vehicle_specs)

    def update_speed(self, tick, leader_speed, relative_position_infront):
        super().base_update_speed(tick)

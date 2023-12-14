import numpy as np
from scipy import signal

class VehicleDynamics:
    def __init__(
        self,
        max_speed,
        max_acceleration,
        max_deceleration,
        mass,
        ss_hp,
        ss_r,
        ss_a
    ):
        self.mass = mass
        self.max_speed = max_speed
        self.max_acceleration = max_acceleration
        self.max_deceleration = max_deceleration
        self.ss_hp = ss_hp
        self.ss_r = ss_r
        # Assume all vehicles have same mass
        self.ss_m = np.array([mass, mass, mass])
        self.ss_a = ss_a


class BidirectionalStateSpace:
    def __init__(
        self,
        order: int,
        total_vehicles: int,
        min_distance: float,
        period: float,
        dynamics: VehicleDynamics
    ):
        self.__order = order
        self.__total_vehicles = total_vehicles
        self.__min_distance = min_distance
        self.__period = period
        self.__dynamics = dynamics

        a11 = -((1 + dynamics.ss_hp) - (-1 + dynamics.ss_hp) * \
                (order % total_vehicles) / (order) * \
                (dynamics.ss_r[1] / dynamics.ss_m[1])
        a12 = dynamics.ss_a[1]
        a21 = -1 / dynamics.ss_m[1]
        b12 = (1 + dynamics.ss_hp) * (dynamics.ss_r[0] / dynamics.ss_m[0])
        b13 = -(-1 + dynamics.ss_hp) * (dynamics.ss_r[2] / dynamics.ss_m[2])
        b15 = dynamics.ss_a[2]
        b22 = 1 / dynamics.ss_m[0]

        self.__A_ct = np.array([[a11, a12], [a21, 0]])
        self.__B_ct = np.array([[-1, b12, b13, 0, b15], [0, b22, 0, 0, 0]])
        self.__C_ct = np.array([[1 / dynamics.mass, 0]])
        self.__D_ct = np.array([[0]])

        # Convert to discrete-time system
        (
            self.__A_dt,
            self.__B_dt,
            self.__C_dt,
            self.__D_dt,
            _
        ) = signal.cont2discrete(
            (self.__A_ct, self.__B_ct, self.__C_ct, self.__D_ct),
            dt=period,
            method="zoh"
        )

    def __get_positioning_error(self, distance, leader_velocity):
        ref_self = self.__get_min_distance(
            leader_velocity, self.__order)
        ref_infront = self.__get_min_distance(
            leader_velocity, self.__order - 1)

        # TODO: Set self.position
        return -((ref_self - self.position) - (ref_infront - distance))

    def __get_min_distance(self, leader_velocity, order):
        speed_in_m_per_s = leader_velocity / 3.6
        margin_in_m = self.__min_distance
        minimal_distance = speed_in_m_per_s * self.period + margin_in_m
        return minimal_distance * order

    def __get_momentum(self, velocity):
        return self.__dynamics.mass * velocity

    def update(
        self,
        distance,
        velocity_self,
        velocity_leader,
        velocity_in_front,
        velocity_behind
    ) -> float:
        """
        Calculates a new desired velocity based on adjacent vehicles.
        """
        delta = self.__get_positioning_error(distance, velocity_leader)
        state = np.array([self.__get_momentum(velocity_self), delta])
        inputs = np.array([
            velocity_leader,
            self.__get_momentum(velocity_in_front),
            self.__get_momentum(velocity_behind),
            self.__get_positioning_error(??, velocity_in_front),
            self.__get_positioning_error(??, velocity_behind)
        ])

        return self.__C_dt @ (self.__A_dt @ state + self.__B_dt @ inputs)

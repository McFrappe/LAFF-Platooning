import numpy as np

class VelocityMapper:
    def __init__(
            self,
            map_file: str,
            polyfit_deg: int,
            idle: int,
            min_forward: int
            max_forward: int,
    ):
        self.__data = np.genfromtxt(
            map_file,
            delimiter=",",
            skip_header=1,
            names=["pwm", "velocity"])
        z = np.polyfit(
            self.__data["velocity"], self.__data["pwm"], polyfit_deg)
        self.__fn = np.poly1d(z)

        self.__idle = idle
        self.__max_forward = max_forward
        self.__min_forward = min_forward
        self.__min_velocity = np.min(self.__data["velocity"])
        self.__max_velocity = np.max(self.__data["velocity"])

    def to_pwm(self, velocity) -> int:
        """
        Converts a velocity into a PWM value, based on the loaded
        Velocity PWM map. The returned PWM is bounded between
        idle and maximum forward, and any velocities smaller or
        larger than the mapped velocities will return these values.
        """
        if velocity < self.__min_velocity:
            return self.__idle

        if velocity > self.__max_velocity:
            return self.__max_forward

        return int(
            min(max(self.__fn(velocity), self.__idle), self.__max_forward))

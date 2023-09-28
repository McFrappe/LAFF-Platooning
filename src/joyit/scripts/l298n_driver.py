import rospy

from common import *
from l298n_pin_config import L298NPinConfig


class L298NDriver:

    def __init__(self, name, config: L298NPinConfig, max_speed=MAX_SPEED_MOTOR):
        """
        Init communication, set default settings, ...
        """
        self.config = config
        self.max_speed = max_speed
        self.name = name

        # TODO: check these settings according to the motors and L298N driver
        self.current_speed = 0
        self.voltage = VOLTAGE_MOTOR
        # cant see how this is relevant to set manually. Should probalby fetch the values through ros or something. But i dont even think there are temperature sensors on the motors lol.
        self.temperature = TEMPERATURE_MOTOR

    def set_speed(self, speed):
        """
        Give a speed that the motor will try to reach.
        """
        if speed <= self.max_speed:
            self.current_speed = speed
        else:
            self.current_speed = self.max_speed

    def set_speed_max(self):
        """
        Set the maximum speed for the motor
        """
        self.current_speed = self.max_speed

    def stop(self):
        """
        Set speed to 0 and thus stop the motor
        """
        self.current_speed = 0

    def get_speed(self):
        """
        Return current speed of the motor
        """
        return self.current_speed

    def get_status(self):
        """
        Get hardware information from the motor
        """
        return {
            'name': self.name,
            'current speed': self.current_speed,
            'voltage': self.voltage,
            'temperature': self.temperature
        }

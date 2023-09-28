import rospy

from std_mgssgs.msg import Int64
from l298n_pin_config import L298NPinConfig

class L298NDriver:

    def __init__(self, name, config: L298NPinConfig, max_speed=100):
        """
        Init communication, set default settings, ...
        """
        self.config = config
        self.max_speed = max_speed
        self.name += name

        # TODO: check these settings according to the motors and L298N driver
        self.current_speed = 0
        self.voltage = 12
        self.temperature = 47

    def set_speed(self, speed):
        """
        Give a speed that the motor will try to reach.
        """
        if speed <= self.max_speed:
            self.current_speed = speed
        else:
            self.current_speed = self.max_speed

    def set_speed_max(self):
        pass

    def stop(self):
        """
        Set speed to 0 and thus stop the motor
        """
        self.current_speed = 0

    def get_speed(self):
        """
        Return current speed
        """
        return self.current_speed

    def get_status(self):
        """
        Get hardware information from the motor
        """
        return {
            'temperature': self.temperature,
            'voltage': self.voltage
        }
import rospy
import RPi.GPIO as GPIO

import common
from l298n_pin_config import L298NPinConfig

class L298NDriver:
    def __init__(self, name, config: L298NPinConfig, max_speed=common.MAX_SPEED_MOTOR):
        """
        Init communication, set default settings, ...
        """
        self.name = name
        self.config = config
        self.current_speed = 0
        self.max_speed = max_speed

        self.setup_pins()

    def setup_pins(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.config.IN1, GPIO.OUT)
        GPIO.setup(self.config.IN2, GPIO.OUT)
        GPIO.setup(self.config.IN3, GPIO.OUT)
        GPIO.setup(self.config.IN4, GPIO.OUT)
        GPIO.setup(self.config.ENA, GPIO.OUT)
        GPIO.setup(self.config.ENB, GPIO.OUT)

        # GPIO.PWM finds which channel is used by the pin number
        self.pwm_a = GPIO.PWM(self.config.ENA, common.PWM_FREQUENCY)
        self.pwm_b = GPIO.PWM(self.config.ENB, common.PWM_FREQUENCY)

        self.pwm_a.start(0)
        self.pwm_b.start(0)

        self.set_direction(common.DIR_FORWARD)

    def set_speed(self, speed: int):
        """
        Give a speed that the motor will try to reach.
        """
        self.current_speed = min(speed, self.max_speed)
        self.pwm_a.ChangeDutyCycle(self.current_speed)
        self.pwm_b.ChangeDutyCycle(self.current_speed)

    def set_direction(self, direction: int):
        if direction == common.DIR_FORWARD:
            GPIO.output(self.config.IN1, 0)
            GPIO.output(self.config.IN2, 1)
            GPIO.output(self.config.IN3, 0)
            GPIO.output(self.config.IN4, 1)
        elif direction == common.DIR_BACKWARD:
            GPIO.output(self.config.IN1, 1)
            GPIO.output(self.config.IN2, 0)
            GPIO.output(self.config.IN3, 1)
            GPIO.output(self.config.IN4, 0)

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
            "name": self.name,
            'current_speed': self.current_speed,
        }

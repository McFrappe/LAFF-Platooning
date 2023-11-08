import rospy
from time import time, sleep

try:
    import RPi.GPIO as GPIO
except:
    import Mock.GPIO as GPIO

class UltrasonicDriver:
    def __init__(self, trigger_pin: int, echo_pin: int):
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin
        self.timeout = rospy.get_param("ULTRASONIC_TIMEOUT_LENGTH")
        self.ping_length = rospy.get_param("ULTRASONIC_PING_PULSE_LENGTH")

        self.min_range = rospy.get_param("ULTRASONIC_MIN_RANGE")
        self.max_range = rospy.get_param("ULTRASONIC_MAX_RANGE")
        self.fov = rospy.get_param("ULTRASONIC_FOV")

        self.__setup_pins()

    def __setup_pins(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.trigger_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)
        GPIO.output(self.trigger_pin, 0)

    def get_distance(self):
        GPIO.output(self.trigger_pin, 1)
        sleep(self.ping_length)
        GPIO.output(self.trigger_pin, 0)

        start_time = time()
        arrival_time = time()
        timeout_start = time()

        while GPIO.input(self.echo_pin) == 0:
            start_time = time()
            if start_time - timeout_start > self.timeout:
                return -1

        while GPIO.input(self.echo_pin) == 1:
            arrival_time = time()
            if start_time - timeout_start > self.timeout:
                return -1

        if start_time != 0 and arrival_time != 0:
            # calculate the difference between start and stop
            duration = arrival_time - start_time

            # multiply with speed of sound (34300 cm/s)
            # and divide by 2 because there and back
            distance = (duration * 34300) / 2

            if distance >= 0:
                return distance
            else:
                return -1
        else:
            return -1

    def get_speed(self):
        start_distance = self.get_distance() * 0.01     # to m conversion
        end_distance = self.get_distance() * 0.01       # to m conversion
        speed = (end_distance - start_distance) / 1.0   # m/s
        return speed

    def cleanup(self):
        GPIO.cleanup()

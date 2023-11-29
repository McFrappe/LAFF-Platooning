import time
import rospy
import RPi.GPIO as GPIO


class ReflectiveSensorDriver:
    """
    Driver for QDRE1113 reflective sensor using digital reading (GPIO).
    """

    def __init__(self, pin, wait_time_us):
        self.pin = pin
        self.wait_time = wait_time_us * 10**3
        GPIO.setmode(GPIO.BOARD)

    def get_value(self):
        """
        Returns the value of the sensor. The value is the time it takes for the sensor to go from HIGH to LOW. The value is in seconds.
        """
        GPIO.setup(self.pin, GPIO.OUT)
        GPIO.output(self.pin, GPIO.HIGH)
        time.sleep(10 / 10**6)
        GPIO.setup(self.pin, GPIO.IN)

        start = time.time_ns()
        while GPIO.input(self.pin) and time.time_ns() - start < self.wait_time:
            pass
        diff = time.time_ns() - start
        return diff / 10**3

    def cleanup(self):
        """
        Cleans up the GPIOs.
        """
        GPIO.cleanup()

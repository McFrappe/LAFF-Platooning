try:
    import RPi.GPIO as GPIO
except:
    import Mock.GPIO as GPIO

class IRArrayDriver:
    def __init__(self, data_pin_left: int, data_pin_middle: int, data_pin_right: int) -> None:
        """
        Constructor for IR array driver.
        The IR array driver is a device that has 3 IR sensors on it.
        The array is used to detect a line on the ground for instance.
        """
        self.pin_left   = data_pin_left
        self.pin_middle = data_pin_middle
        self.pin_right  = data_pin_right

        self.__setup_pins()

    def __setup_pins(self) -> None:
        """
        Setup the pins for the IR array driver.
        """
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin_left, GPIO.IN)
        GPIO.setup(self.pin_middle, GPIO.IN)
        GPIO.setup(self.pin_right, GPIO.IN)

    def __to_int(self, value) -> int:
        """
        Ensure that the value is an integer.
        """
        return 1 if value else 0

    def get_values(self):
        """
        Perform a digital read on ALL IR sensors.
        """
        return [
            self.__to_int(GPIO.input(self.pin_left)),
            self.__to_int(GPIO.input(self.pin_middle)),
            self.__to_int(GPIO.input(self.pin_right))
        ]

    def cleanup(self):
        """
        Cleanup the GPIO pins.
        """
        GPIO.cleanup()
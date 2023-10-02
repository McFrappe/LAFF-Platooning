import RPi.GPIO as GPIO

class IRArrayDriver:
    """
    Class for IR array driver. The IR array driver is a device that has 3 IR sensors on it. The array is used to detect a line on the ground for instance.
    """
    pin_left_value      = 0
    pin_middle_value    = 0
    pin_right_value     = 0

    def __init__(self, data_pin_left, data_pin_middle, data_pin_right) -> None:
        """
        Constructor for IR array driver. The IR array driver is a device that has 3 IR sensors on it. The array is used to detect a line on the ground for instance.
        """
        self.pin_left   = data_pin_left
        self.pin_middle = data_pin_middle
        self.pin_right  = data_pin_right

        self.__setup_pins()
        self.get_array_values()

    def __setup_pins(self) -> None:
        """
        Setup the pins for the IR array driver.
        """
        GPIO.setup(self.pin_left, GPIO.IN)
        GPIO.setup(self.pin_middle, GPIO.IN)
        GPIO.setup(self.pin_right, GPIO.IN)

    def read_array_sensors(self) -> None:
        """
        Perform a digital read on ALL IR sensors.
        """
        self.pin_left_value     = GPIO.DigitalRead(self.pin_left)
        self.pin_middle         = GPIO.DigitalRead(self.pin_middle)
        self.pin_right_value    = GPIO.DigitalRead(self.pin_right)

    def read_left_sensor(self) -> None:
        """
        Perform a digital read on the LEFT IR sensor.
        """
        self.pin_left_value = GPIO.DigitalRead(self.pin_left)

    def read_middle_sensor(self) -> None:
        """
        Perform a digital read on the MIDDLE IR sensor.
        """
        self.pin_middle = GPIO.DigitalRead(self.pin_middle)

    def read_right_sensor(self) -> None:
        """
        Perform a digital read on the RIGHT IR sensor.
        """
        self.pin_right_value = GPIO.DigitalRead(self.pin_right)

    def get_left_value(self) -> int:
        """
        Get the value stored from last digital read of the LEFT IR sensor.
        """
        return self.pin_left_value

    def get_middle_value(self) -> int:
        """
        Get the value stored from last digital read of the MIDDLE IR sensor.
        """
        return self.pin_middle_value

    def get_right_value(self) -> int:
        """
        Get the value stored from last digital read of the RIGHT IR sensor.
        """
        return self.pin_right_value

    def get_array_values(self) -> dict[str, int]:
        """
        Get all the values stored from last digital read of the IR sensors.
        """
        return {
            "left":     self.pin_left_value,
            "middle":   self.pin_middle_value,
            "right":    self.pin_right_value
            }
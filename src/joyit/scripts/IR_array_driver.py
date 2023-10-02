import gpio_wrapper as GPIO

class IRArrayDriver:
    """
    Class for IR array driver. The IR array driver is a device that has 3 IR sensors on it. The array is used to detect a line on the ground for instance.
    """
    def __init__(self, data_pin_left: int, data_pin_middle: int, data_pin_right: int) -> None:
        """
        Constructor for IR array driver. The IR array driver is a device that has 3 IR sensors on it. The array is used to detect a line on the ground for instance.
        """
        self.pin_left   = data_pin_left
        self.pin_middle = data_pin_middle
        self.pin_right  = data_pin_right

        self.__pin_left_value      = 0
        self.__pin_middle_value    = 0
        self.__pin_right_value     = 0

        self.__setup_pins()
        self.__read_array_sensors()

    #region Private methods for IR array driver
    def __setup_pins(self) -> None:
        """
        Setup the pins for the IR array driver.
        """
        GPIO.setup(self.pin_left, GPIO.IN)
        GPIO.setup(self.pin_middle, GPIO.IN)
        GPIO.setup(self.pin_right, GPIO.IN)

    def __read_array_sensors(self) -> None:
        """
        Perform a digital read on ALL IR sensors.
        """
        self.__pin_left_value   = GPIO.DigitalRead(self.pin_left)
        self.__pin_middle_value = GPIO.DigitalRead(self.pin_middle)
        self.__pin_right_value  = GPIO.DigitalRead(self.pin_right)

    def __read_left_sensor(self) -> None:
        """
        Perform a digital read on the LEFT IR sensor.
        """
        self.pin_left_value = GPIO.DigitalRead(self.pin_left)

    def __read_middle_sensor(self) -> None:
        """
        Perform a digital read on the MIDDLE IR sensor.
        """
        self.pin_middle_value = GPIO.DigitalRead(self.pin_middle)

    def __read_right_sensor(self) -> None:
        """
        Perform a digital read on the RIGHT IR sensor.
        """
        self.pin_right_value = GPIO.DigitalRead(self.pin_right)
    #endregion

    #region Public methods for IR array driver
    def get_left_value(self) -> int:
        """
        Get the value stored from last digital read of the LEFT IR sensor.
        """
        return self.__pin_left_value

    def get_middle_value(self) -> int:
        """
        Get the value stored from last digital read of the MIDDLE IR sensor.
        """
        return self.__pin_middle_value

    def get_right_value(self) -> int:
        """
        Get the value stored from last digital read of the RIGHT IR sensor.
        """
        return self.__pin_right_value

    def get_all_values(self):
        """
        Get all the values stored from last digital read of the IR sensors.
        """
        return {
            "left":     self.__pin_left_value,
            "middle":   self.__pin_middle_value,
            "right":    self.__pin_right_value
        }

    def update_value(self, sensor: str) -> None:
        """
        Update the value of a specific IR sensor.
        param: sensor: str, left, middle or right
        """
        if sensor == "left":
            self.__read_left_sensor()
        elif sensor == "middle":
            self.__read_middle_sensor()
        elif sensor == "right":
            self.__read_right_sensor()
        else:
            raise ValueError("Invalid sensor name. Valid sensor names are left, middle and right.")

    def update_values(self) -> None:
        """
        Update the values of the IR sensors.
        """
        self.__read_array_sensors()

    def cleanup(self):
        """
        Cleanup the GPIO pins.
        """
        GPIO.cleanup()
    #endregion
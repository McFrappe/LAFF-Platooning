import pigpio


class PWMDriver:
    def __init__(self, out_pin: int, freq: int):
        """
        Constructor for a general PWM driver.
        """
        self.__pin = out_pin
        self.__pi = pigpio.pi()
        self.__freq = freq
        self.__setup_pins()

    def __setup_pins(self) -> None:
        self.__pi.set_mode(self.__pin, pigpio.OUTPUT)

    def set_pwm(self, pwm: int) -> None:
        self.__pi.hardware_PWM(self.__pin, self.__freq, pwm)

    def cleanup(self):
        self.__pi.stop()

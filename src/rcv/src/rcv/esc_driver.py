import rospy
import time
import pigpio

class ESCDriver:
    def __init__(self, out_pin: int):
        """
        Constructor for ESC driver.
        The ESC is connected to the electric motor och the RC vehicle.
        The ESC is used to control the elecric motor.
        """
        self.__pin = out_pin
        self.__pi = pigpio.pi()
        self.__freq = rospy.get_param("PWM_FREQUENCY_MOTOR")
        self.__setup_pins()

    def __setup_pins(self) -> None:
        """
        Setup the pins for the  array driver.
        """
        self.__pi.set_mode(self.__pin, pigpio.OUTPUT)

    def set_pwm(self, pwm: int) -> None:
        """
        Give a speed that the motor will try to reach.
        """
        self.__pi.hardware_PWM(self.__pin, self.__freq, pwm)

    def cleanup(self):
        self.__pi.stop()

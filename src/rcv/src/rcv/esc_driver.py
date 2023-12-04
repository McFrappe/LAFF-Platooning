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
        self.__pin = 13
        self.__pi = pigpio.pi()
        self.__setup_pins()

    def __setup_pins(self) -> None:
        """
        Setup the pins for the  array driver.
        """
        self.__pi.set_mode(self.__pin, pigpio.OUTPUT)
        self.__pi.set_PWM_range(self.__pin, 100)
        self.__pi.set_PWM_frequency(self.__pin, rospy.get_param("PWM_FREQUENCY_MOTOR"))
        self.__pi.set_PWM_dutycycle(self.__pin, 0)

    def set_pwm(self, pwm: int) -> None:
        """
        Give a speed that the motor will try to reach.
        """
        rospy.loginfo(f"PWM: {pwm}, FREQ: {self.__pi.get_PWM_frequency(self.__pin)}, DC: {self.__pi.get_PWM_dutycycle(self.__pin)}")
        self.__pi.set_PWM_dutycycle(self.__pin, pwm)

    def cleanup(self):
        self.__pi.stop()

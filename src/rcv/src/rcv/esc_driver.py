import rospy
import time

try:
    import RPi.GPIO as GPIO
except:
    import Mock.GPIO as GPIO

class ESCDriver:
    def __init__(self, out_pin: int):
        """
        Constructor for ESC driver.
        The ESC is connected to the electric motor och the RC vehicle.
        The ESC is used to control the elecric motor.
        """
        self.__pin = out_pin
        self.__setup_pins()

    def __setup_pins(self) -> None:
        """
        Setup the pins for the  array driver.
        """
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.__pin, GPIO.OUT)
        self.__pwm = GPIO.PWM(self.__pin, rospy.get_param("PWM_FREQUENCY_MOTOR"))
        self.__pwm.start(0)

    def set_pwm(self, pwm: int) -> None:
        """
        Give a speed that the motor will try to reach.
        """
        self.__pwm.ChangeDutyCycle(pwm)

    def cleanup(self):
        GPIO.cleanup()

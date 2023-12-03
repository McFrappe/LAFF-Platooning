import rospy
import time

try:
    import RPi.GPIO as GPIO
except:
    import Mock.GPIO as GPIO

class ESCDriver:
    def __init__(self,
            out_pin: int,
            max_forward=rospy.get_param("MAX_FORWARD_MOTOR"),
            max_reverse=rospy.get_param("MAX_REVERSE_MOTOR"),
            idle=rospy.get_param("IDLE_MOTOR")) -> None:
        """
        Constructor for ESC driver.
        The ESC is connected to the electric motor och the RC vehicle.
        The ESC is used to control the elecric motor.
        """
        self.__pin  = out_pin
        self.__idle = idle
        self.__max_forward = max_forward
        self.__max_reverse = max_reverse
        self.__setup_pins()

    def __setup_pins(self) -> None:
        """
        Setup the pins for the  array driver.
        """
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.__pin, GPIO.OUT)
        self.__pwm = GPIO.PWM(self.__pin, rospy.get_param("PWM_FREQUENCY_MOTOR"))
        self.__pwm.start(0)

    def set_speed(self, speed: int) -> None:
        """
        Give a speed that the motor will try to reach.
        """
        new_speed = max(self.__max_reverse, min(speed, self.__max_forward))
        self.__pwm.ChangeDutyCycle(new_speed)

    def cleanup(self):
        """
        Cleanup the GPIO pins.
        """
        self.set_speed(self.__idle)
        GPIO.cleanup()

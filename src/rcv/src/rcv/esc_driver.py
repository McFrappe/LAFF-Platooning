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
        self.pin  = out_pin
        self.current_speed = idle
        self.max_forward = max_forward
        self.max_reverse = max_reverse
        self.setup_pins()

    def setup_pins(self) -> None:
        """
        Setup the pins for the IR array driver.
        """
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pin, rospy.get_param("PWM_FREQUENCY_MOTOR"))

        self.pwm.start(0) # Start with 0% duty cycle

    def calibrate(self) -> None:
        """
        Calibrate the ESC. This is done by setting the ESC to the current speed for 10 seconds.
        """
        self.pwm.ChangeDutyCycle(self.current_speed)
        time.sleep(10)

    def set_speed(self, speed: int) -> None:
        """
        Give a speed that the motor will try to reach.
        """
        self.current_speed = max(self.max_reverse, min(speed, self.max_forward))
        self.pwm.ChangeDutyCycle(self.current_speed)

    def get_speed(self) -> int:
        """
        Return current speed of the motor
        """
        return self.current_speed

    def get_status(self):
        """
        Get hardware information from the motor
        """
        return {
            'current_speed': self.current_speed,
        }

    def cleanup(self):
        """
        Cleanup the GPIO pins.
        """
        GPIO.cleanup()
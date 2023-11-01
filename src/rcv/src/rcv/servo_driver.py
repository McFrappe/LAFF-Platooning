import rospy

try:
    import RPi.GPIO as GPIO
except:
    import Mock.GPIO as GPIO

class ServoDriver:
    def __init__(self, 
            out_pin: int, 
            max_right=rospy.get_param("MAX_RIGHT_ANGLE"), 
            max_left=rospy.get_param("MAX_LEFT_ANGLE"), 
            zero=rospy.get_param("ZERO_ANGLE")) -> None:
        """
        Constructor for Servo driver.
        The servo motor is connected to the steering rack on the RC vehicle. 
        The servo motor is used to steer the RC vehicle.
        """
        self.pin  = out_pin
        self.current_angle = zero
        self.max_right = max_right
        self.max_left = max_left
        self.setup_pins()

    def setup_pins(self) -> None:
        """
        Setup the pins for the IR array driver.
        """
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pin, rospy.get_param("PWM_FREQUENCY_MOTOR"))

        self.pwm.start(0)

    def set_angle(self, angle: int) -> None:
        self.current_angle = max(self.max_left, min(angle, self.max_right))
        self.pwm.ChangeDutyCycle(self.current_angle)

    def get_angle(self) -> int:
        """
        Return current speed of the motor
        """
        return self.current_angle

    def get_status(self):
        """
        Get hardware information from the motor
        """
        return {
            'current_speed':    self.current_angle,
        }

    def cleanup(self):
        """
        Cleanup the GPIO pins.
        """
        GPIO.cleanup()
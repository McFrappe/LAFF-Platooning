#!/usr/bin/env python3
import rospy

from std_msgs.msg import Int32

from common.pwm_driver import PWMDriver

class ServoController:
    """
    Controller for the servo.
    """
    def __init__(self):
        self.__id = rospy.get_param("VEHICLE_ID")
        self.__driver = PWMDriver(
            rospy.get_param("SERVO_PIN"),
            rospy.get_param("PWM_FREQUENCY_MOTOR"))

        self.__zero = rospy.get_param("ZERO_ANGLE")
        self.__max_right = rospy.get_param("MAX_RIGHT_ANGLE")
        self.__max_left = rospy.get_param("MAX_LEFT_ANGLE")

        rospy.Subscriber(
            f"{self.__id}/steering_angle",
            Int32,
            self.__callback_steering_angle)

    def __callback_steering_angle(self, msg: Int32):
        """
        Callback function for the steering angle.
        """
        new_angle = max(self.__max_left, min(msg.data, self.__max_right))
        self.__driver.set_pwm(new_angle)

    def stop(self):
        """
        Stop the servo.
        """
        self.__driver.set_pwm(self.__zero)
        self.__driver.cleanup()

if __name__ == "__main__":
    rospy.init_node("servo_node", anonymous=True)
    controller = ServoController()
    rospy.on_shutdown(controller.stop)

    rospy.loginfo("Servo controller node started.")
    rospy.spin()

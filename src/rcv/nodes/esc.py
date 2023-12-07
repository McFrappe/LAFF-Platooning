#!/usr/bin/env python3
import rospy

from std_msgs.msg import Int32, Bool

from common.pwm_driver import PWMDriver

class ESCController:
    """
    Controller for the ESC.
     """
    def __init__(self):
        self.__id = rospy.get_param("VEHICLE_ID")
        self.__message_queue_size = rospy.get_param("MESSAGE_QUEUE_SIZE")

        self.__idle = rospy.get_param("IDLE_MOTOR")
        self.__min_forward = rospy.get_param("MIN_FORWARD_MOTOR")
        self.__max_forward = rospy.get_param("MAX_FORWARD_MOTOR")
        self.__max_reverse = rospy.get_param("MAX_REVERSE_MOTOR")
        self.__calibrated = False

        self.__driver = PWMDriver(
            rospy.get_param("MOTOR_PIN"),
            rospy.get_param("PWM_FREQUENCY_MOTOR"))

        self.__calibrated_publisher = rospy.Publisher(
            f"{self.__id}/esc_calibrated",
            Bool,
            queue_size=self.__message_queue_size)

        rospy.Subscriber(f"{self.__id}/pwm", Int32, self.__callback_pwm)
        rospy.Timer(
            rospy.Duration(1),
            self.__callback_start_calibration,
            oneshot=True)
        rospy.Timer(
            rospy.Duration(2),
            self.__callback_broadcast_calibration,
            oneshot=True)

    def __callback_start_calibration(self, event):
        self.__driver.set_pwm(self.__idle)

    def __callback_broadcast_calibration(self, event):
        self.__calibrated = True
        self.__calibrated_publisher.publish(True)

    def __callback_pwm(self, msg: Int32):
        """
        Callback function for the pwm.
        """
        if not self.__calibrated:
            return

        new_pwm = max(self.__max_reverse, min(msg.data, self.__max_forward))
        self.__driver.set_pwm(new_pwm)

    def stop(self):
        """
        Stop the ESC.
        """
        self.__driver.set_pwm(self.__idle)
        self.__driver.cleanup()

if __name__ == "__main__":
    rospy.init_node("esc_node", anonymous=True)
    controller = ESCController()
    rospy.on_shutdown(controller.stop)

    rospy.loginfo("ESC controller node started.")
    rospy.spin()

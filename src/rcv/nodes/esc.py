#!/usr/bin/env python3
import rospy

from std_msgs.msg import Int32, Bool

from rcv.esc_driver import ESCDriver

class ESCController:
    """
    Controller for the ESC.
     """
    def __init__(self):
        self.__driver = ESCDriver(
            out_pin=rospy.get_param("MOTOR_PIN"))

        self.__id = rospy.get_param("VEHICLE_ID")
        self.__message_queue_size = rospy.get_param("MESSAGE_QUEUE_SIZE")
        self.__idle = rospy.get_param("IDLE_MOTOR")
        self.__calibrated = False

        self.__calibrated_publisher = rospy.Publisher(
            f"{self.__id}/esc_calibrated",
            Bool,
            queue_size=self.__message_queue_size)

        rospy.Subscriber(f"{self.__id}/pwm", Int32, self.__callback_pwm)
        rospy.Timer(
            rospy.Duration(1), self.__callback_start_calibration, oneshot=True)
        rospy.Timer(
            rospy.Duration(11), self.__callback_stop_calibration, oneshot=True)

    def __callback_start_calibration(self, event):
        self.__driver.set_pwm(self.__idle)

    def __callback_stop_calibration(self, event):
        self.__driver.set_pwm(self.__idle)
        self.__calibrated = True
        self.__calibrated_publisher.publish(True)

    def __callback_pwm(self, msg: Int32):
        """
        Callback function for the pwm.
        """
        if not self.__calibrated:
            return
        self.__driver.set_pwm(msg.data)

    def stop(self):
        """
        Stop the ESC.
        """
        self.__driver.cleanup()

if __name__ == "__main__":
    rospy.init_node("esc_node", anonymous=True)
    controller = ESCController()
    rospy.on_shutdown(controller.stop)

    rospy.loginfo("ESC controller node started.")
    rospy.spin()

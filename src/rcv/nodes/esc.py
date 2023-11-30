#!/usr/bin/env python3
import rospy

from std_msgs.msg import Int32

from rcv.esc_driver import ESCDriver

class ESCController:
    """
    Controller for the ESC.
     """
    def __init__(self):
        self.__id = rospy.get_param("VEHICLE_ID")
        self.__driver = ESCDriver(
            out_pin=rospy.get_param("MOTOR_PIN"))

        self.__idle = rospy.get_param("IDLE_MOTOR")
        self.__speed = self.__idle
        self.__calibrated = False

        rospy.Subscriber(f"{self.__id}/speed", Int32, self.__callback_speed)
        rospy.Timer(
            rospy.Duration(1), self.__callback_start_calibration, oneshot=True)
        rospy.Timer(
            rospy.Duration(2), self.__callback_stop_calibration, oneshot=True)

    def __callback_start_calibration(self, event):
        self.__driver.start_calibration()

    def __callback_stop_calibration(self, event):
        self.__driver.stop_calibration()

    def __callback_speed(self, msg: Int32):
        """
        Callback function for the speed.
        """
        self.__driver.set_speed(msg.data)

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

#!/usr/bin/env python3
import rospy

from std_msgs.msg import Int32

from rcv.esc_driver import ESCDriver

class ESCController:
    """
    Controller for the ESC.
     """
    def __init__(self):
        self.driver = ESCDriver(
            out_pin=rospy.get_param("MOTOR_PIN"))

        self.speed = rospy.get_param("IDLE_MOTOR")
        self.driver.calibrate()
        self.setup_service()

    def setup_service(self):
        """
        Setup the service for the ESC.
        """
        rospy.Subscriber("vehicle/speed", Int32, self.callback_speed)

    def callback_speed(self, msg: Int32):
        """
        Callback function for the speed.
        """
        self.speed = msg.data
        self.driver.set_speed(self.speed)
        rospy.loginfo(f"Status of speed is {self.driver.get_status()}")

    def stop(self):
        """
        Stop the ESC.
        """
        self.driver.set_speed(rospy.get_param("IDLE_MOTOR"))
        self.driver.cleanup()

if __name__ == "__main__":
    rospy.init_node("esc_node")
    controller = ESCController()
    rospy.on_shutdown(controller.stop)

    rospy.loginfo("ESC controller node started.")
    rospy.spin()
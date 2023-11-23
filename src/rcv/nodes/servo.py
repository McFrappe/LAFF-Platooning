#!/usr/bin/env python3
import rospy

from std_msgs.msg import Int32

from rcv.servo_driver import ServoDriver

class ServoController:
    """
    Controller for the servo.
    """

    def __init__(self):
        self.id = rospy.get_param("VEHICLE_ID")
        self.driver = ServoDriver(
            out_pin=rospy.get_param("SERVO_PIN"))

        self.angle = rospy.get_param("ZERO_ANGLE")
        self.setup_service()

    def setup_service(self):
        """
        Setup the service for the servo steering.
        """
        rospy.Subscriber(f"{self.id}/steering_angle", Int32, self.callback_steering_angle)

    def callback_steering_angle(self, msg: Int32):
        """
        Callback function for the steering angle.
        """
        self.angle = msg.data
        self.driver.set_angle(self.angle)
        #rospy.loginfo(f"Status of steering angle is {self.driver.get_status()}")

    def stop(self):
        """
        Stop the servo.
        """
        self.driver.set_angle(rospy.get_param("ZERO_ANGLE"))
        self.driver.cleanup()

if __name__ == "__main__":
    rospy.init_node("servo_node", anonymous=True)
    controller = ServoController()
    rospy.on_shutdown(controller.stop)

    rospy.loginfo("Servo controller node started.")
    rospy.spin()

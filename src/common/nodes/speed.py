#!/usr/bin/env python3
import rospy

from std_msgs.msg import UInt8MultiArray

from common.reflective_sensor_driver import ReflectiveSensorDriver


class ReflectiveSensorController:
    def __init__(self):
        self.driver = ReflectiveSensorDriver(
            rospy.get_param("SPEED_SENSOR_PIN"))

        self.publisher = rospy.Publisher(
            "vehicle/velocity",
            UInt8MultiArray,
            queue_size=rospy.get_param("MESSAGE_QUEUE_SIZE"))

        rospy.Timer(rospy.Duration(rospy.get_param(
            "SPEED_PUBLISH_PERIOD")), self.publish_line_position)

    def publish_line_position(self, event):
        """
        Publishes the current line position to the vehicle/line_follower topic.
        """

        # TODO: convert to km/h or m/s
        speed = self.driver.get_value()
        rospy.loginfo(f"Speed (km/h): {speed}")
        self.publisher.publish(UInt8MultiArray(data=speed))

    def cleanup(self):
        """
        Cleans up the GPIOs.
        """
        self.driver.cleanup()


if __name__ == "__main__":
    rospy.init_node("speed_node")
    controller = ReflectiveSensorController()
    rospy.on_shutdown(controller.cleanup)

    rospy.loginfo("Speed node started.")
    rospy.spin()

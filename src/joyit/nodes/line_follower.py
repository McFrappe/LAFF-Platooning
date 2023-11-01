#!/usr/bin/env python3
import rospy

from std_msgs.msg import UInt8MultiArray

from joyit.IR_array_driver import IRArrayDriver

class LineFollowerController:
    def __init__(self):
        self.driver = IRArrayDriver(
            rospy.get_param("IR_LEFT"),
            rospy.get_param("IR_MIDDLE"),
            rospy.get_param("IR_RIGHT"))

        self.publisher = rospy.Publisher(
            "vehicle/line_follower",
            UInt8MultiArray,
            queue_size=rospy.get_param("MESSAGE_QUEUE_SIZE"))

        rospy.Timer(rospy.Duration(rospy.get_param("LINE_FOLLOWER_PUBLISH_PERIOD")), self.publish_line_position)

    def publish_line_position(self, event):
        """
        Publishes the current line position to the vehicle/line_follower topic.
        """
        position = self.driver.get_values()
        rospy.loginfo(f"Line position: {position}")
        self.publisher.publish(UInt8MultiArray(data=position))

    def cleanup(self):
        """
        Cleans up the GPIOs.
        """
        self.driver.cleanup()


if __name__ == "__main__":
    rospy.init_node("line_follower_node")
    controller = LineFollowerController()
    rospy.on_shutdown(controller.cleanup)

    rospy.loginfo("Line Follower node started.")
    rospy.spin()

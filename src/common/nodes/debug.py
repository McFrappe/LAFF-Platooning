#!/usr/bin/env python3
import rospy

from sensor_msgs.msg import Range
from std_msgs.msg import String, Int32, Float32

class DebugController:
    def __init__(self):
        self.__id = rospy.get_param("VEHICLE_ID")
        self.__publish_period = rospy.get_param("DEBUG_PUBLISH_PERIOD")
        self.__message_queue_size = rospy.get_param("MESSAGE_QUEUE_SIZE")

        self.__steering_angle = 0
        self.__distance = 0
        self.__pwm = 0
        self.__velocity = 0
        self.__pid = 0

        self.__debug_publisher = rospy.Publisher(
            f"{self.__id}/debug",
            String,
            queue_size=self.__message_queue_size)

        self.__steering_angle_subscriber = rospy.Subscriber(
            f"{self.__id}/steering_angle",
            Int32,
            self.__callback_steering_angle,
            queue_size=self.__message_queue_size)

        self.__distance_subscriber = rospy.Subscriber(
            f"{self.__id}/distance",
            Range,
            self.__callback_distance,
            queue_size=self.__message_queue_size)

        self.__pwm_subscriber = rospy.Subscriber(
            f"{self.__id}/pwm",
            Int32,
            self.__callback_pwm,
            queue_size=self.__message_queue_size)

        self.__pid_subscriber = rospy.Subscriber(
            f"{self.__id}/pid",
            Float32,
            self.__callback_pid,
            queue_size=self.__message_queue_size)

        self.__velocity_subscriber = rospy.Subscriber(
            f"{self.__id}/velocity",
            Float32,
            self.__callback_velocity,
            queue_size=self.__message_queue_size)

        rospy.Timer(rospy.Duration(self.__publish_period), self.__publish_debug)

    def __callback_steering_angle(self, msg: Int32):
        self.__steering_angle = msg.data

    def __callback_distance(self, msg: Range):
        self.__distance = msg.range

    def __callback_pwm(self, msg: Int32):
        self.__pwm = msg.data

    def __callback_pid(self, msg: Float32):
        self.__pid = msg.data

    def __callback_velocity(self, msg: Float32):
        self.__velocity = msg.data

    def __publish_debug(self, event):
        msg = f"[{self.__id}]"
        msg += f" steering angle: {self.__steering_angle},"
        msg += f" distance: {self.__distance},"
        msg += f" pwm: {self.__pwm},"
        msg += f" PID: {self.__pid},"
        msg += f" velocity: {self.__velocity}"
        self.__debug_publisher.publish(msg)
        rospy.loginfo(msg)

    def cleanup(self):
        self.__steering_angle_subscriber.unregister()
        self.__distance_subscriber.unregister()
        self.__pwm_subscriber.unregister()
        self.__pid_subscriber.unregister()
        self.__velocity_subscriber.unregister()


if __name__ == "__main__":
    rospy.init_node("debug_node", anonymous=True)
    controller = DebugController()
    rospy.on_shutdown(controller.cleanup)

    rospy.loginfo("Debug node started.")
    rospy.spin()

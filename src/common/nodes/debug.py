#!/usr/bin/env python3
import time
import rospy

from sensor_msgs.msg import Range
from std_msgs.msg import String, Int32, Float32, Bool

class DebugController:
    def __init__(self):
        self.__id = rospy.get_param("VEHICLE_ID")
        self.__publish_period = rospy.get_param("DEBUG_PUBLISH_PERIOD")
        self.__message_queue_size = rospy.get_param("MESSAGE_QUEUE_SIZE")

        self.__out_path = rospy.get_param("DEBUG_OUT_PATH")
        self.__write_to_file = rospy.get_param("DEBUG_WRITE_TO_FILE")

        self.__steering_angle = 0
        self.__distance = 0
        self.__pwm = 0
        self.__velocity = 0
        self.__control = 0
        self.__has_target = False
        self.__target_center_offset = 0

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

        self.__control_subscriber = rospy.Subscriber(
            f"{self.__id}/control",
            Float32,
            self.__callback_control,
            queue_size=self.__message_queue_size)

        self.__velocity_subscriber = rospy.Subscriber(
            f"{self.__id}/velocity",
            Float32,
            self.__callback_velocity,
            queue_size=self.__message_queue_size)

        self.__has_target_subscriber = rospy.Subscriber(
            f"{self.__id}/has_target",
            Bool,
            self.__callback_has_target,
            queue_size=self.__message_queue_size)

        self.__target_center_offset_subscriber = rospy.Subscriber(
            f"{self.__id}/target_center_offset",
            Int32,
            self.__callback_target_center_offset,
            queue_size=self.__message_queue_size)

        rospy.Timer(rospy.Duration(self.__publish_period), self.__publish_debug)

        if self.__write_to_file:
            self.__write_header()

    def __write_header(self):
        # Write CSV header and clear any previous data in the file.
        with open(self.__out_path, "w") as f:
            f.write("time,pwm,distance,angle,control,velocity,target,target_center_offset\n")

    def __flush_data(self):
        """
        Saves debug data to file.
        """
        values_str = ",".join([
            str(time.time_ns()),
            str(self.__pwm),
            str(self.__distance),
            str(self.__steering_angle),
            str(self.__control),
            str(self.__velocity),
            str(self.__has_target),
            str(self.__target_center_offset),
        ])
        with open(self.__out_path, "a") as f:
            f.write(f"{values_str}\n")

    def __callback_steering_angle(self, msg: Int32):
        self.__steering_angle = msg.data

    def __callback_distance(self, msg: Range):
        self.__distance = msg.range

    def __callback_pwm(self, msg: Int32):
        self.__pwm = msg.data

    def __callback_control(self, msg: Float32):
        self.__control = msg.data

    def __callback_velocity(self, msg: Float32):
        self.__velocity = msg.data

    def __callback_has_target(self, msg: Bool):
        self.__has_target = msg.data

    def __callback_target_center_offset(self, msg: Int32):
        self.__target_center_offset = msg.data

    def __publish_debug(self, event):
        msg = f"[{self.__id}]"
        msg += f" angle: {self.__steering_angle},"
        msg += f" dist: {self.__distance:.2f},"
        msg += f" pwm: {self.__pwm},"
        msg += f" ctrl: {self.__control:.2f},"
        msg += f" vel: {self.__velocity:.2f},"
        msg += f" has_target: {self.__has_target},"
        msg += f" center_offset: {self.__target_center_offset}"
        self.__debug_publisher.publish(msg)
        rospy.loginfo(msg)

        if self.__write_to_file:
            self.__flush_data()

    def cleanup(self):
        self.__steering_angle_subscriber.unregister()
        self.__distance_subscriber.unregister()
        self.__pwm_subscriber.unregister()
        self.__control_subscriber.unregister()
        self.__velocity_subscriber.unregister()


if __name__ == "__main__":
    rospy.init_node("debug_node", anonymous=True)
    controller = DebugController()
    rospy.on_shutdown(controller.cleanup)

    rospy.loginfo("Debug node started.")
    rospy.spin()

#!/usr/bin/env python3
import rospy
from simple_pid import PID
import numpy as np

from std_msgs.msg import Int32, Bool
from sensor_msgs.msg import RegionOfInterest
from pixy2_msgs.msg import PixyData, PixyBlock, PixyResolution

class ObjectFollowerController:
    def __init__(self):
        self.__id = rospy.get_param("VEHICLE_ID")
        self.__max_right = rospy.get_param("MAX_RIGHT_ANGLE")
        self.__max_left = rospy.get_param("MAX_LEFT_ANGLE")
        self.__zero = rospy.get_param("ZERO_ANGLE")
        self.__update_period = rospy.get_param("OBJECT_FOLLOWER_PERIOD")
        self.__message_queue_size = rospy.get_param("MESSAGE_QUEUE_SIZE")
        self.__nr_of_blocks_to_collect = rospy.get_param(
            "OBJECT_FOLLOWER_BLOCKS_PER_UPDATE")

        self.__resolution_x = 315
        self.__detected_blocks: list[PixyBlock] = []
        self.__collected_blocks: list[PixyBlock] = []
        self.__collected_blocks_count = 0

        self.__pid_kp = rospy.get_param("OBJECT_FOLLOWER_PID_KP")
        self.__pid_ki = rospy.get_param("OBJECT_FOLLOWER_PID_KI")
        self.__pid_kd = rospy.get_param("OBJECT_FOLLOWER_PID_KD")
        self.__pid_setpoint = rospy.get_param("OBJECT_FOLLOWER_PID_SETPOINT")
        self.__pid_min = rospy.get_param("OBJECT_FOLLOWER_PID_MIN")
        self.__pid_max = rospy.get_param("OBJECT_FOLLOWER_PID_MAX")
        self.__pid = pid = PID(
            self.__pid_kp,
            self.__pid_ki,
            self.__pid_kd,
            setpoint=self.__pid_setpoint)

        self.__pid.sample_time = (
            self.__update_period / self.__nr_blocks_to_collect)
        self.__pid.output_limits = (self.__pid_min, self.__pid_max)

        self.__has_target_publisher = rospy.Publisher(
            f"{self.__id}/has_target",
            Bool,
            queue_size=self.__message_queue_size)

        self.__steering_angle_publisher = rospy.Publisher(
            f"{self.__id}/steering_angle",
            Int32,
            queue_size=self.__message_queue_size)

        self.__resolution_subscriber = rospy.Subscriber(
            f"{self.__id}/pixy2_resolution",
            PixyResolution,
            self.__callback_resolution,
            queue_size=self.__message_queue_size)

        self.__blocks_subscriber = rospy.Subscriber(
            f"{self.__id}/block_data",
            PixyData,
            self.__callback_blocks,
            queue_size=self.__message_queue_size)

        rospy.Timer(rospy.Duration(self.__update_period), self.__update)

    def __callback_resolution(self, data: PixyResolution):
        self.__resolution_x = data.width

    def __callback_blocks(self, data: PixyData):
        self.__detected_blocks = data.blocks

    def __calculate_center_offset(self, blocks):
        """
        Calculates the average offset of a list of objects from the center of the
        image. A negative offset indicates that the object is too
        far to the left, and a positive offset that the object is
        too far to the right.
        """
        avg_width = sum([b.roi.width for b in blocks]) / len(blocks)
        avg_x_offset = sum([b.roi.x_offset for b in blocks]) / len(blocks)
        center_pos = self.__resolution_x / 2

        return avg_width, ((avg_x_offset + (avg_width / 2)) - center_pos)

    def __update(self, event):
        """
        Updates the steering angle based on object(s) detected
        by the Pixy2 camera.
        """
        if self.__collected_blocks_count < self.__nr_of_blocks_to_collect:
            self.__collected_blocks_count += 1

            if len(self.__detected_blocks) != 0:
                self.__collected_blocks.append(self.__detected_blocks[0])

            return

        new_angle = self.__zero
        has_target = len(self.__collected_blocks) != 0

        # No blocks detected during the period, reset
        if has_target:
            avg_width, center_offset = self.__calculate_center_offset(
                self.__collected_blocks)
            error = pid(center_offset)
            new_angle = int(np.interp(
                error,
                [self.__pid_min, self.__pid_max],
                [self.__max_left, self.__max_right])

        self.__has_target_publisher.publish(has_target)
        self.__steering_angle_publisher.publish(new_angle)
        self.__collected_blocks = []
        self.__collected_blocks_count = 0

    def cleanup(self):
        self.__blocks_subscriber.unregister()


if __name__ == "__main__":
    rospy.init_node("object_follower_node", anonymous=True)
    controller = ObjectFollowerController()
    rospy.on_shutdown(controller.cleanup)

    rospy.loginfo("Object Follower node started.")
    rospy.spin()

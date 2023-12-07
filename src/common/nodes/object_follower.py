#!/usr/bin/env python3
import rospy
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
        self.__fine_adjust_divider = rospy.get_param(
            "OBJECT_FOLLOWER_RESOLUTION_FINE_ADJUST_DIVIDER")
        self.__large_adjust_divider = rospy.get_param(
            "OBJECT_FOLLOWER_RESOLUTION_LARGE_ADJUST_DIVIDER")
        self.__large_adjust_threshold = rospy.get_param(
            "OBJECT_FOLLOWER_LARGE_ADJUST_THRESHOLD_PX")
        self.__max_width_of_resolution_modifier = rospy.get_param(
            "OBJECT_FOLLOWER_MAX_WIDTH_OF_RESOLUTION_MODIFIER")
        self.__detected_blocks: list[PixyBlock] = []
        self.__collected_blocks: list[PixyBlock] = []
        self.__collected_blocks_count = 0

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

    def __calculate_horizontal_offset(self, blocks):
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

        # No blocks detected during the period, reset
        if len(self.__collected_blocks) == 0:
            self.__has_target_publisher.publish(False)
            self.__steering_angle_publisher.publish(self.__zero)
            self.__collected_blocks_count = 0
            return

        new_angle = self.__zero
        avg_width, hoffset = self.__calculate_horizontal_offset(self.__collected_blocks)

        if avg_width >= self.__max_width_of_resolution_modifier * self.__resolution_x:
            self.__steering_angle_publisher.publish(self.__zero)
            return

        if abs(hoffset) <= self.__large_adjust_threshold:
            max_value = int(self.__resolution_x / self.__fine_adjust_divider)
        else:
            max_value = int(self.__resolution_x / self.__large_adjust_divider)

        if hoffset < 0:
            # Turn left
            new_angle = int(np.interp(
                (self.__resolution_x / 2) - abs(hoffset),
                [0, max_value],
                [self.__zero, self.__max_left]))
        elif hoffset > 0:
            # Turn right
            new_angle = int(np.interp(
                abs(hoffset),
                [0, max_value],
                [self.__zero, self.__max_right]))

        self.__has_target_publisher.publish(True)
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

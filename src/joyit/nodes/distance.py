#!/usr/bin/env python3
import rospy

from sensor_msgs.msg import Range

from joyit.ultrasonic_driver import UltrasonicDriver

class DistanceController:
    def __init__(self):
        self.__current_reading = 0
        self.__average_distance = 0.0
        self.__readings_per_publish = rospy.get_param("ULTRASONIC_SAMPLES_PER_PUBLISH")

        self.driver = UltrasonicDriver(
            rospy.get_param("ULTRASONIC_TRIGGER"),
            rospy.get_param("ULTRASONIC_ECHO"))

        self.distance_publisher = rospy.Publisher(
            "vehicle/distance",
            Range,
            queue_size=rospy.get_param("MESSAGE_QUEUE_SIZE"))

        rospy.Timer(rospy.Duration(rospy.get_param("DISTANCE_PUBLISH_PERIOD")),
                    self.publish_current_distance)

    def create_range_message(self, distance):
        """
        Creates a Range message with the given distance value.
        """
        range_msg = Range()
        range_msg.header.stamp = rospy.Time.now()
        range_msg.header.frame_id = "/base_link"
        range_msg.radiation_type = Range.ULTRASOUND
        range_msg.field_of_view = self.driver.fov
        range_msg.min_range = self.driver.min_range
        range_msg.max_range = self.driver.max_range
        range_msg.range = distance
        return range_msg

    def reset_reading_state(self):
        """
        Resets the current reading state to start a new set of readings.
        """
        self.__current_reading = 0
        self.__average_distance = 0

    def publish_current_distance(self, event):
        """
        Publishes the current distance to the vehicle/distance topic.
        """
        if self.__current_reading < self.__readings_per_publish:
            self.__average_distance += self.driver.get_distance()
            self.__current_reading += 1
            return

        distance = self.__average_distance / self.__readings_per_publish
        rospy.loginfo(f"Distance: {distance:.2f} cm (avg. of {self.__readings_per_publish} samples)")

        self.distance_publisher.publish(self.create_range_message(distance))

        # Restore starting reading state
        self.reset_reading_state()

    def stop(self):
        """
        Stops the distance node.
        """
        self.distance_publisher.unregister()
        self.driver.cleanup()


if __name__ == "__main__":
    rospy.init_node("distance_node")
    controller = DistanceController()
    rospy.on_shutdown(controller.stop)

    rospy.loginfo("Distance node started.")
    rospy.spin()

#!/usr/bin/env python3
import rospy
import numpy as np

from std_msgs.msg import Float32

from common.reflective_sensor_driver import ReflectiveSensorDriver


class VelocityController:
    def __init__(self):
        self.id = rospy.get_param("VEHICLE_ID")
        self.wheel_radius_cm = rospy.get_param("VEHICLE_WHEEL_RADIUS_CM")

        self.wait_time_us = rospy.get_param("VELOCITY_WAIT_TIME_US")
        self.reflectance_threshold_us = rospy.get_param(
            "VELOCITY_REFLECTANCE_THRESHOLD_US")

        self.driver = ReflectiveSensorDriver(
            rospy.get_param("REFLECTIVE_SENSOR_PIN"), self.wait_time_ms)

        self.publish_period = rospy.get_param("VELOCITY_PUBLISH_PERIOD")

        self.publisher = rospy.Publisher(
            f"{self.id}/velocity",
            Float32,
            queue_size=rospy.get_param("MESSAGE_QUEUE_SIZE"))

        self.readings = []
        self.readings_per_publish_period = self.publish_period / \
            (self.wait_time_us / 10**6)

        rospy.Timer(rospy.Duration(self.publish_period), self.publish_velocity)

    def get_velocity(self):
        """
        Returns the velocity of the vehicle in m/s.
        """
        while len(self.readings) < self.readings_per_publish_period:
            self.readings.append(self.driver.get_value())

        rotations_per_second = (1 / self.publish_period) / (
            lambda x: x < self.reflectance_threshold_us, self.readings)

        return (self.wheel_radius_cm / 100) * \
            (2 * np.pi / 60) * rotations_per_second

    def publish_velocity(self, event):
        """
        Publishes the current velocity to the velocity topic.
        """
        velocity_ms = self.get_velocity()
        rospy.loginfo(f"Velocity (m/s): {velocity_ms}")
        self.publisher.publish(Float32(data=velocity_ms))
        self.readings = []

    def cleanup(self):
        """
        Cleans up the GPIOs.
        """
        self.driver.cleanup()


if __name__ == "__main__":
    rospy.init_node("velocity_node", anonymous=True)
    controller = VelocityController()
    rospy.on_shutdown(controller.cleanup)

    rospy.loginfo("Velocity node started.")
    rospy.spin()

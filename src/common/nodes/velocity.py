#!/usr/bin/env python3
import time
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
        self.tapes_per_rotation = rospy.get_param(
            "VELOCITY_TAPES_PER_ROTATION")

        self.driver = ReflectiveSensorDriver(
            rospy.get_param("REFLECTIVE_SENSOR_PIN"), self.wait_time_us)

        self.publish_period = rospy.get_param("VELOCITY_PUBLISH_PERIOD")

        self.publisher = rospy.Publisher(
            f"{self.id}/velocity",
            Float32,
            queue_size=rospy.get_param("MESSAGE_QUEUE_SIZE"))

        self.readings = []
        self.prev_time = None
        self.outside = False
        rospy.Timer(rospy.Duration(self.wait_time_us / 10**6), self.get_velocity)
        rospy.Timer(rospy.Duration(self.publish_period), self.publish_velocity)

    def get_velocity(self, event):
        """
        Returns the velocity of the vehicle in m/s.
        """
        val = self.driver.get_value()
        if val <= self.reflectance_threshold_us:
            if self.prev_time is None:
                self.prev_time = time.time_ns()
            elif self.outside is True:
                # Convert to us and save
                self.readings.append((time.time_ns() - self.prev_time)/10**9)
                self.prev_time = None
            self.outside = False
        else:
            self.outside = True

    def publish_velocity(self, event):
        """
        Publishes the current velocity to the velocity topic.
        """
        if len(self.readings) == 0:
            velocity_ms = 0
        else:
            mean_diff = np.mean(self.readings)
            # If we have more than one piece of tape per rotation, the distance traveled
            # per reading (time between tape, no tape, and tape) only corresponds to
            # a n:th of the total circumference of the wheel.
            distance_traveled = (self.wheel_radius_cm / 100) / self.tapes_per_rotation
            velocity_ms = (distance_traveled * 2 * np.pi) / mean_diff

        velocity_kmh = velocity_ms * 3.6
        self.publisher.publish(Float32(data=velocity_kmh))
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

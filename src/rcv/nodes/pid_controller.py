#!/usr/bin/env python3
import rospy
import numpy as np

from sensor_msgs.msg import Range
from std_msgs.msg import Int32, Float32

from controller.pid import PID

class PIDController:
    def __init__(self):
        self.__period = rospy.get_param("PID_CONTROL_PERIOD")
        self.__id = rospy.get_param("VEHICLE_ID")
        self.__pid_platooning = PID(
            rospy.get_param("K_PP"),
            rospy.get_param("K_PI"),
            rospy.get_param("K_PD"),
            self.__period, # FIXME: should this be DISTANCE_PUBLISH_PERIOD?
            rospy.get_param("PID_PMIN"),
            rospy.get_param("PID_PMAX"))

        self.__pid_speed = PID(
            rospy.get_param("K_SP"),
            rospy.get_param("K_SI"),
            rospy.get_param("K_SD"),
            self.__period, # FIXME: should this be VELOCITY_PUBLISH_PERIOD?
            rospy.get_param("PID_SMIN"),
            rospy.get_param("PID_SMAX"))

        self.__velocity_min = rospy.get_param("VELOCITY_MIN")
        self.__velocity_max = rospy.get_param("VELOCITY_MAX")
        self.__margin_in_m = rospy.get_param("PID_PLATOONING_MARGIN_M")

        self.__max_forward = rospy.get_param("MAX_FORWARD_MOTOR")
        self.__max_reverse = rospy.get_param("MAX_REVERSE_MOTOR")
        self.__idle = rospy.get_param("IDLE_MOTOR")
        self.__max_right = rospy.get_param("MAX_RIGHT_ANGLE")
        self.__max_left = rospy.get_param("MAX_LEFT_ANGLE")
        self.__zero = rospy.get_param("ZERO_ANGLE")

        self.__steering_angle = self.__zero
        self.__current_velocity = 0
        self.__reference_velocity = 0
        self.__current_pwm = self.__idle
        self.__current_distance = 0

        self.__message_queue_size = rospy.get_param("MESSAGE_QUEUE_SIZE")

        self.speed_publisher = rospy.Publisher( # TODO: change to pwm_speed
            f"{self.__id}/speed",
            Int32,
            queue_size=self.__message_queue_size)

        self.pid_publisher = rospy.Publisher( # TODO: change to velocity or something
            f"{self.__id}/pid",
            Float32,
            queue_size=self.__message_queue_size)

        self.steering_angle_publisher = rospy.Publisher(
            f"{self.__id}/steering_angle",
            Int32,
            queue_size=self.__message_queue_size)

        self.distance_subscriber = rospy.Subscriber(
            f"{self.__id}/distance",
            Range,
            self.__callback_distance,
            queue_size=self.__message_queue_size)

        self.velocity_subscriber = rospy.Subscriber(
            f"{self.__id}/velocity",
            Float32,
            self.__callback_velocity,
            queue_size=self.__message_queue_size)

        rospy.Timer(rospy.Duration(self.__period), self.__perform_step)

    def __callback_distance(self, msg: Range):
        """
        Callback for the distance subscriber.
        """
        self.__current_distance = msg.range

    def __callback_velocity(self, msg: Float32):
        """
        Callback for the velocity subscriber.
        """
        self.__current_velocity = msg.data

    def __min_distance(self):
        speed_in_m_per_s = self.__current_velocity/3.6
        # TODO: should depend on speed and vehicle
        self.min_distance = speed_in_m_per_s * self.__period * 2 + self.__margin_in_m
        return self.min_distance

    def __perform_step(self, event):
        distance_error = self.__current_distance - self.__min_distance()
        platoon_control_output = self.__pid_platooning.update(distance_error)
        desired_velocity = self.__current_velocity+platoon_control_output
        self.__reference_velocity = min(
            max(desired_velocity, self.__velocity_min), self.__velocity_max)

        speed_error = self.__reference_velocity - self.__current_velocity
        pwm_control_output = self.__pid_speed.update(speed_error)
        desired_pwm = self.__current_pwm + pwm_control_output
        self.__current_pwm = min(
            max(desired_pwm, self.__idle), self.__max_forward)

        self.speed_publisher.publish(self.__current_pwm)
        self.pid_publisher.publish(self.__reference_velocity)

    def stop(self):
        self.speed_publisher.publish(self.__idle)

if __name__ == "__main__":
    rospy.init_node("pid_controller_node", anonymous=True)
    controller = PIDController()
    rospy.on_shutdown(controller.stop)

    rospy.loginfo("PID controller node started.")
    rospy.spin()

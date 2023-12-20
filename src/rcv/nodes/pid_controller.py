#!/usr/bin/env python3
import rospy
import numpy as np

from sensor_msgs.msg import Range
from std_msgs.msg import Int32, Float32, Bool

from controller.pid import PID

class PIDController:
    def __init__(self):
        self.__id = rospy.get_param("VEHICLE_ID")
        self.__period = rospy.get_param("PID_CONTROL_PERIOD")
        self.__pid_platooning = PID(
            rospy.get_param("K_PP"),
            rospy.get_param("K_PI"),
            rospy.get_param("K_PD"),
            self.__period, # FIXME: should this be DISTANCE_PUBLISH_PERIOD?
            rospy.get_param("PID_PMIN"),
            rospy.get_param("PID_PMAX"))

        self.__pid_pwm = PID(
            rospy.get_param("K_SP"),
            rospy.get_param("K_SI"),
            rospy.get_param("K_SD"),
            self.__period, # FIXME: should this be VELOCITY_PUBLISH_PERIOD?
            rospy.get_param("PID_SMIN"),
            rospy.get_param("PID_SMAX"))

        self.__margin_in_m = rospy.get_param("PID_PLATOONING_MARGIN_M")
        self.__message_queue_size = rospy.get_param("MESSAGE_QUEUE_SIZE")
        self.__stop_vehicle_if_no_target = rospy.get_param(
            "VEHICLE_STOP_IF_NO_OBJECT_VISIBLE")

        self.__min_forward = rospy.get_param("MIN_FORWARD_MOTOR")
        self.__max_forward = rospy.get_param("MAX_FORWARD_MOTOR")
        self.__max_reverse = rospy.get_param("MAX_REVERSE_MOTOR")
        self.__idle = rospy.get_param("IDLE_MOTOR")

        self.__has_target = False
        self.__esc_calibrated = False
        self.__desired_pwm = self.__idle
        self.__current_pwm = self.__idle
        self.__current_distance = 0
        self.__current_velocity = 0
        self.__current_leader_velocity = 0
        self.__is_leader = self.__id == "vehicle_0"

        self.pwm_publisher = rospy.Publisher(
            f"{self.__id}/pwm",
            Int32,
            queue_size=self.__message_queue_size)

        self.control_publisher = rospy.Publisher(
            f"{self.__id}/control",
            Float32,
            queue_size=self.__message_queue_size)

        self.distance_subscriber = rospy.Subscriber(
            f"{self.__id}/distance",
            Range,
            self.__callback_distance,
            queue_size=self.__message_queue_size)

        if not self.__is_leader:
            self.leader_velocity_subscriber = rospy.Subscriber(
                "/vehicle_0/velocity",
                Float32,
                self.__callback_leader_velocity,
                queue_size=self.__message_queue_size)

        self.velocity_subscriber = rospy.Subscriber(
            f"{self.__id}/velocity",
            Float32,
            self.__callback_velocity,
            queue_size=self.__message_queue_size)

        self.esc_calibrated_subscriber = rospy.Subscriber(
            f"{self.__id}/esc_calibrated",
            Bool,
            self.__callback_esc_calibrated,
            queue_size=self.__message_queue_size)

        self.has_target_subscriber = rospy.Subscriber(
            f"{self.__id}/has_target",
            Bool,
            self.__callback_has_target,
            queue_size=self.__message_queue_size)

        rospy.Timer(rospy.Duration(self.__period), self.__perform_step)

    def __callback_esc_calibrated(self, msg: Bool):
        """
        Callback for when the ESC has been calibrated.
        """
        self.__esc_calibrated = msg.data

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

    def __callback_leader_velocity(self, msg: Float32):
        """
        Callback for the leader velocity subscriber.
        """
        rospy.loginfo(f"Leader velocity: {msg.data}")
        self.__current_leader_velocity = msg.data

    def __callback_has_target(self, msg: Bool):
        self.__has_target = msg.data

    def __min_distance(self):
        speed_in_m_per_s = self.__current_velocity / 3.6
        return speed_in_m_per_s * self.__period * 2 + self.__margin_in_m

    def __perform_step(self, event):
        if not self.__esc_calibrated:
            return

        # Only apply PID controller if we have an object to follow,
        # and we have enabled the flag in the launch file.
        if not self.__has_target and self.__stop_vehicle_if_no_target:
            self.__current_pwm = self.__idle
        else:
            distance_error = self.__current_distance - self.__min_distance()
            platoon_control_output = self.__pid_platooning.update(distance_error)

            if platoon_control_output < 0:
                self.__desired_pwm = self.__max_reverse
            elif platoon_control_output == 0:
                self.__desired_pwm = self.__idle
            else:
                self.__desired_pwm = max(
                    self.__desired_pwm + platoon_control_output,
                    self.__min_forward)

            self.__current_pwm = int(min(
                max(self.__desired_pwm, self.__max_reverse),
                self.__max_forward))

        self.pwm_publisher.publish(self.__current_pwm)
        self.control_publisher.publish(self.__desired_pwm)

    def stop(self):
        self.pwm_publisher.publish(self.__idle)

if __name__ == "__main__":
    rospy.init_node("pid_controller_node", anonymous=True)
    controller = PIDController()
    rospy.on_shutdown(controller.stop)

    rospy.loginfo("PID controller node started.")
    rospy.spin()

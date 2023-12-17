#!/usr/bin/env python3
import os
import sys
import rospy
import numpy as np

from std_msgs.msg import Float32, Int32, Bool

class VelocityPWMMapper:
    def __init__(self):
        self.__id = rospy.get_param("VEHICLE_ID")
        self.__out_path = rospy.get_param("MAPPER_OUT_PATH")
        self.__step_size = rospy.get_param("MAPPER_PWM_STEP_SIZE")
        self.__nr_samples = rospy.get_param("MAPPER_SAMPLES_PER_PWM_STEP")

        self.__idle = rospy.get_param("IDLE_MOTOR")
        self.__min_forward = rospy.get_param("MIN_FORWARD_MOTOR")
        self.__max_forward = rospy.get_param("MAX_FORWARD_MOTOR")

        self.__message_queue_size = rospy.get_param("MESSAGE_QUEUE_SIZE")
        self.__velocity_update_period = rospy.get_param(
            "VELOCITY_PUBLISH_PERIOD")

        self.__started_mapping = False
        self.__current_pwm = self.__idle
        self.__samples = []
        self.__current_sample = 0

        self.__velocity_subscriber = rospy.Subscriber(
            f"{self.__id}/velocity",
            Float32,
            self.__callback_velocity,
            queue_size=self.__message_queue_size)

        self.__esc_calibrated_subscriber = rospy.Subscriber(
            f"{self.__id}/esc_calibrated",
            Bool,
            self.__callback_esc_calibrated,
            queue_size=self.__message_queue_size)

        self.__pwm_publisher = rospy.Publisher(
            f"{self.__id}/pwm",
            Int32,
            queue_size=self.__message_queue_size)

        # Write CSV header and clear any previous data in the file.
        self.__write_header()

    def __write_header(self):
        with open(self.__out_path, "w") as f:
            f.write("pwm,velocity")

    def __flush_samples(self):
        """
        Calculates the mean velocity of the samples and saves it to file.
        """
        with open(self.__out_path, "a") as f:
            f.write(f"{self.__current_pwm},{np.mean(self.__samples)}\n")

    def __increase_pwm(self):
        if self.__current_pwm >= self.__max_forward:
            self.__pwm_publisher.publish(self.__idle)
            sys.exit(0)

        if self.__current_pwm == self.__idle:
            self.__current_pwm = self.__min_forward
        else:
            self.__current_pwm = min(
                self.__current_pwm + self.__step_size, self.__max_forward)

        self.__pwm_publisher.publish(self.__current_pwm)

    def __callback_esc_calibrated(self, msg: Bool):
        self.__started_mapping = msg.data

    def __callback_velocity(self, msg: Float32):
        if not self.__started_mapping:
            return

        if self.__current_sample < self.__nr_samples:
            if self.__current_sample == 0:
                self.__increase_pwm()
            else:
                self.__samples.append(msg.data)

            self.__current_sample += 1
        else:
            self.__flush_samples()
            self.__samples = []
            self.__current_sample = 0

    def cleanup(self):
        self.__velocity_subscriber.unregister()
        self.__esc_calibrated_subscriber.unregister()


if __name__ == "__main__":
    rospy.init_node("velocity_pwm_mapper", anonymous=True)
    node = VelocityPWMMapper()
    rospy.on_shutdown(node.cleanup)

    rospy.loginfo("Velocity PWM mapper started.")
    rospy.spin()

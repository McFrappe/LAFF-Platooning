#!/usr/bin/env python3
import rospy
import numpy as np

from std_msgs.msg import Int32
from sensor_msgs.msg import Joy


class JoystickController:
    def __init__(self):
        self.id = rospy.get_param("VEHICLE_ID")
        self.max_forward = rospy.get_param("MAX_FORWARD_MOTOR")
        self.max_reverse = rospy.get_param("MAX_REVERSE_MOTOR")
        self.idle = rospy.get_param("IDLE_MOTOR")
        self.current_pwm = self.idle
        self.old_pwm = 0
        self.max_right=rospy.get_param("MAX_RIGHT_ANGLE")
        self.max_left=rospy.get_param("MAX_LEFT_ANGLE")
        self.zero=rospy.get_param("ZERO_ANGLE")
        self.steering_angle = self.zero
        self.old_angle = 0
        self.message_queue_size = rospy.get_param("MESSAGE_QUEUE_SIZE")

        self.pwm_publisher = rospy.Publisher(
            f"{self.id}/pwm",
            Int32,
            queue_size=self.message_queue_size)

        self.steering_angle_publisher = rospy.Publisher(
            f"{self.id}/steering_angle",
            Int32,
            queue_size=self.message_queue_size)

        self.controller_subscriber = rospy.Subscriber(
            "/joy",
            Joy,
            self.__callback_controller,
            queue_size=self.message_queue_size)

        rospy.Timer(
            rospy.Duration(rospy.get_param("JOYSTICK_CONTROL_PERIOD")),
            self.perform_step)

    def __callback_controller(self, data: Joy):
        """
        Callback for the joy subscriber.
        """
        # https://github.com/naoki-mizuno/ds4_driver/blob/humble-devel/ds4_driver_msgs/msg/Status.msg
        # Left: 1.0, Right: -1.0, Up: 1.0, Down: -1.0
        steer_axis_x = data.axes[0]
        speed_axis_y = data.axes[3]
        reverse_pwm = data.axes[4]
        forward_pwm = data.axes[5]

        # Buttons (0: Not pressed, 1: Pressed)
        button_dpad_left = data.buttons[-4]
        button_dpad_right = data.buttons[-2]

        # Change zero angle after trim input deom dpad
        if button_dpad_left == 1:
            self.zero = max(self.zero - 1, self.max_left)
        elif button_dpad_right == 1:
            self.zero = min(self.zero + 1, self.max_right)

        if forward_pwm > 0:
            self.current_pwm = int(
                np.interp(forward_pwm, [0, 1], [self.idle, self.max_forward]))
        else:
            self.current_pwm = int(
                np.interp(reverse_pwm, [0, 1], [self.idle, self.max_reverse]))

        self.steering_angle = int(
            np.interp(steer_axis_x, [-1, 1], [self.max_right, self.max_left]))

    def perform_step(self, event):
        self.pwm_publisher.publish(self.current_pwm)
        self.steering_angle_publisher.publish(self.steering_angle)

        if self.current_pwm != self.old_pwm:
            self.old_pwm = self.current_pwm

        if self.steering_angle != self.old_angle:
            self.old_angle = self.steering_angle

    def stop(self):
        self.pwm_publisher.publish(self.idle)
        self.controller_subscriber.unregister()

if __name__ == "__main__":
    rospy.init_node("joystick_controller_node", anonymous=True)
    controller = JoystickController()
    rospy.on_shutdown(controller.stop)

    rospy.loginfo("Joystick controller node started.")
    rospy.spin()

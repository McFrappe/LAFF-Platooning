#!/usr/bin/env python3
import rospy
import numpy as np

from std_msgs.msg import Int32
from sensor_msgs.msg import Joy


class JoystickController:
    def __init__(self):
        self.max_forward = rospy.get_param("MAX_FORWARD_MOTOR")
        self.max_reverse = rospy.get_param("MAX_REVERSE_MOTOR")
        self.idle = rospy.get_param("IDLE_MOTOR")
        self.current_speed = self.idle
        self.old_speed = 0
        self.max_right=rospy.get_param("MAX_RIGHT_ANGLE")
        self.max_left=rospy.get_param("MAX_LEFT_ANGLE")
        self.zero=rospy.get_param("ZERO_ANGLE")
        self.steering_angle = self.zero
        self.old_angle = 0
        self.message_queue_size = rospy.get_param("MESSAGE_QUEUE_SIZE")

        self.speed_publisher = rospy.Publisher("vehicle/speed",Int32, queue_size=self.message_queue_size)
        self.steering_angle_publisher = rospy.Publisher("vehicle/steering_angle",Int32, queue_size=self.message_queue_size)

        self.controller_subscriber = rospy.Subscriber(
            "/joy",
            Joy,
            self.__callback_controller,
            queue_size=self.message_queue_size)

        rospy.Timer(rospy.Duration(rospy.get_param("JOYSTICK_CONTROL_PERIOD")), self.perform_step)

    def __callback_controller(self, data: Joy):
        """
        Callback for the joy subscriber.
        """
        #rospy.loginfo(f"Data from controller -> {data}")
        # https://github.com/naoki-mizuno/ds4_driver/blob/humble-devel/ds4_driver_msgs/msg/Status.msg
        # Left: 1.0, Right: -1.0, Up: 1.0, Down: -1.0
        steer_axis_x = data.axes[0] 
        speed_axis_y = data.axes[3] 
        reverse_speed = data.axes[4]
        forward_speed = data.axes[5]
        
        # Buttons (0: Not pressed, 1: Pressed)
        button_dpad_left = data.buttons[-4]
        button_dpad_right = data.buttons[-2]
        
        # Change zero angle after trim input deom dpad
        if button_dpad_left == 1:
            self.zero = max(self.zero - 1, self.max_left)
        elif button_dpad_right == 1:
            self.zero = min(self.zero + 1, self.max_right)

        if forward_speed > 0:
            self.current_speed = int(np.interp(forward_speed, [0, 1], [self.idle, self.max_forward]))
        else:
            self.current_speed = int(np.interp(reverse_speed, [0, 1], [self.idle, self.max_reverse]))

        #self.current_speed = int(np.interp(speed_axis_y, [-1, 1], [self.idle, self.max_forward]))
        # TODO: Might need to check and map left and right separately 
        #self.steering_angle = int(np.interp(steer_axis_x, [-1, 1], [self.max_right, self.max_left]))
        self.steering_angle = int(np.interp(steer_axis_x, [-1, 1], [self.zero + 30, self.zero - 30]))

    def perform_step(self, event):
        self.speed_publisher.publish(self.current_speed)
        self.steering_angle_publisher.publish(self.steering_angle)

        if self.current_speed != self.old_speed:
            self.old_speed = self.current_speed
            rospy.loginfo(f"Status of speed is {self.current_speed}")

        if self.steering_angle != self.old_angle:
            self.old_angle = self.steering_angle
            rospy.loginfo(f"Status of steering angle is {self.steering_angle}")

    def stop(self):
        self.speed_publisher.publish(self.idle)
        self.controller_subscriber.unregister()

if __name__ == "__main__":
    rospy.init_node("joystick_controller_node")
    controller = JoystickController()
    rospy.on_shutdown(controller.stop)

    rospy.loginfo("Joystick controller node started.")
    rospy.spin()

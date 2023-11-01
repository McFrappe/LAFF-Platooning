#!/usr/bin/env python3
import rospy

from std_msgs.msg import Int32

class SimpleController:
    def __init__(self):
        self.max_forward = rospy.get_param("MAX_FORWARD_MOTOR")
        self.max_reverse = rospy.get_param("MAX_REVERSE_MOTOR")
        self.idle = rospy.get_param("IDLE_MOTOR")
        self.current_speed = self.idle
        self.max_right=rospy.get_param("MAX_RIGHT_ANGLE")
        self.max_left=rospy.get_param("MAX_LEFT_ANGLE")
        self.zero=rospy.get_param("ZERO_ANGLE")
        self.steering_angle = self.zero
        self.message_queue_size = rospy.get_param("MESSAGE_QUEUE_SIZE")

        self.speed_publisher = rospy.Publisher("vehicle/speed",Int32, queue_size=self.message_queue_size)
        self.steering_angle_publisher = rospy.Publisher("vehicle/steering_angle",Int32, queue_size=self.message_queue_size)

        rospy.Timer(rospy.Duration(rospy.get_param("TEST_PERIOD")),self.perform_step)

    def perform_step(self, event):
        #self.current_speed = min(self.current_speed + 1, self.max_forward)
        self.speed_publisher.publish(self.current_speed)
        rospy.loginfo(f"Status of speed is {self.current_speed}")
        if self.steering_angle == 30:
            self.steering_angle = 80
        else:
            self.steering_angle = self.steering_angle - 1
        self.steering_angle_publisher.publish(self.steering_angle)
        rospy.loginfo(f"Status of steering angle is {self.steering_angle}")

    def stop(self):
        self.speed_publisher.publish(self.idle)

if __name__ == "__main__":
    rospy.init_node("simple_controller_node")
    controller = SimpleController()
    rospy.on_shutdown(controller.stop)

    rospy.loginfo("Simple controller node started.")
    rospy.spin()
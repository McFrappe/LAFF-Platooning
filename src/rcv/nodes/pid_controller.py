#!/usr/bin/env python3
import rospy

from std_msgs.msg import Int32
from sensor_msgs.msg import Range

from controller.pid import PID

class PIDController:
    def __init__(self):
        self.__pid = PID(
            rospy.get_param("K_P"),
            rospy.get_param("K_I"),
            rospy.get_param("K_D"),
            rospy.get_param("PID_REFERENCE"))

        self.__max_forward = rospy.get_param("MAX_FORWARD_MOTOR")
        self.__max_reverse = rospy.get_param("MAX_REVERSE_MOTOR")
        self.__idle = rospy.get_param("IDLE_MOTOR")
        self.__max_right = rospy.get_param("MAX_RIGHT_ANGLE")
        self.__max_left = rospy.get_param("MAX_LEFT_ANGLE")
        self.__zero = rospy.get_param("ZERO_ANGLE")

        self.__steering_angle = self.__zero
        self.__current_speed = self.__idle
        self.__current_distance = 0 # TODO: remember to update this

        self.__message_queue_size = rospy.get_param("MESSAGE_QUEUE_SIZE")

        self.speed_publisher = rospy.Publisher("vehicle/speed", Int32, queue_size=self.__message_queue_size)
        self.steering_angle_publisher = rospy.Publisher("vehicle/steering_angle", Int32, queue_size=self.__message_queue_size)
        self.distance_subscriber = rospy.Subscriber(
            "vehicle/distance",
            Range,
            self.__callback_distance,
            queue_size=self.__message_queue_size)

        rospy.Timer(rospy.Duration(rospy.get_param("PLATOONING_PERIOD")), self.__perform_step)

    def __callback_distance(self, data: Range):
        """
        Callback for the distance subscriber.
        """
        self.__current_distance = data.range

    def __perform_step(self, event):
        updated_control = self.__pid.update(self.__current_distance)
        # TODO: Convert into the correct unit?
        rospy.loginfo(f"PID controller output for distance {self.__current_distance}: {updated_control}")
        self.current_speed = int(min(self.__idle + updated_control, self.__max_forward))
        self.speed_publisher.publish(self.current_speed)

    def stop(self):
        self.speed_publisher.publish(self.__idle)

if __name__ == "__main__":
    rospy.init_node("pid_controller_node")
    controller = PIDController()
    rospy.on_shutdown(controller.stop)

    rospy.loginfo("PID controller node started.")
    rospy.spin()
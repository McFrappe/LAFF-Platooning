import rospy

from sensor_msgs.msg import Range
from std_msgs.msg import Int32, UInt8MultiArray

from controller.pid import my_cool_controller

class PlatooningController:
    def __init__(self):
        self.__max_speed = rospy.get_param("MAX_SPEED_MOTOR")
        self.__min_distance_threshold = rospy.get_param("MINIMUM_DISTANCE_THRESHOLD")
        self.__message_queue_size = rospy.get_param("MESSAGE_QUEUE_SIZE")
        self.__turn_right_key = rospy.get_param("TURN_RIGHT")
        self.__turn_left_key = rospy.get_param("TURN_LEFT")
        self.__no_turn_key = rospy.get_param("NO_TURN")

        self.__current_distance = 0
        self.__line_follower = [0, 0, 0]

        self.speed_publisher = rospy.Publisher(
            "vehicle/speed",
            Int32,
            queue_size=self.__message_queue_size)
        self.direction_publisher = rospy.Publisher(
            "vehicle/direction",
            Int32,
            queue_size=self.__message_queue_size)
        self.turn_publisher = rospy.Publisher(
            "vehicle/turn",
            Int32,
            queue_size=self.__message_queue_size)

        self.distance_subscriber = rospy.Subscriber(
            "vehicle/distance",
            Range,
            self.__callback_distance,
            queue_size=self.__message_queue_size)
        self.line_follower_subscriber = rospy.Subscriber(
            "vehicle/line_follower",
            UInt8MultiArray,
            self.__callback_line_follower,
            queue_size=self.__message_queue_size)

        rospy.Timer(rospy.Duration(
            rospy.get_param("PLATOONING_PERIOD")),
            self.__perform_step)

    def __callback_distance(self, data: Range):
        """
        Callback for the distance subscriber.
        """
        self.__current_distance = data.range

    def __callback_line_follower(self, data: UInt8MultiArray):
        """
        Callback for the line follower subscriber.
        """
        # data is a sequence of hexadecimals.
        self.__line_follower = [x for x in data.data]

    def __perform_step(self, event):
        """
        Performs one step of the platooning algorithm.
        """
        if self.__line_follower == [1, 1, 0]:
            self.speed_publisher.publish(self.__max_speed)
            self.turn_publisher.publish(self.__turn_right_key)
        elif self.__line_follower == [1, 0, 0]:
            self.speed_publisher.publish(int(self.__max_speed / 2))
            self.turn_publisher.publish(self.__turn_right_key)
        elif self.__line_follower == [0, 1, 1]:
            self.speed_publisher.publish(self.__max_speed)
            self.turn_publisher.publish(self.__turn_left_key)
        elif self.__line_follower == [0, 0, 1]:
            self.speed_publisher.publish(int(self.__max_speed / 2))
            self.turn_publisher.publish(self.__turn_left_key)
        else:
            self.speed_publisher.publish(self.__max_speed)
            self.turn_publisher.publish(self.__no_turn_key)

        if self.__current_distance < self.__min_distance_threshold:
            self.speed_publisher.publish(0)
        else:
            self.speed_publisher.publish(self.__max_speed)

    def stop(self):
        """
        Stops the platooning node.
        """
        self.distance_subscriber.unregister()


if __name__ == "__main__":
    rospy.init_node("platooning_node")
    controller = PlatooningController()
    rospy.on_shutdown(controller.stop)

    rospy.loginfo("Platooning node started.")
    rospy.spin()

import rospy

from std_msgs.msg import Int32
from std_srvs.srv import Trigger
from sensor_msgs.msg import Range

from controller.pid import my_cool_controller

class PlatooningController:
    def __init__(self):
        rospy.wait_for_service("vehicle_turn_left")
        rospy.wait_for_service("vehicle_turn_right")
        rospy.wait_for_service("vehicle_is_on_line")
        rospy.wait_for_service("vehicle_is_not_on_line")
        rospy.wait_for_service("vehicle_is_left_of_line")
        rospy.wait_for_service("vehicle_is_right_of_line")
        rospy.wait_for_service("vehicle_is_slightly_left_of_line")
        rospy.wait_for_service("vehicle_is_slightly_right_of_line")

        self.__current_distance = 0
        self.__current_relative_velocity = 0

        self.speed_publisher = rospy.Publisher(
            "vehicle/speed",
            Int32,
            queue_size=rospy.get_param("MESSAGE_QUEUE_SIZE"))
        self.direction_publisher = rospy.Publisher(
            "vehicle/direction",
            Int32,
            queue_size=rospy.get_param("MESSAGE_QUEUE_SIZE"))

        self.distance_subscriber = rospy.Subscriber(
            "vehicle/distance",
            Range,
            self.__callback_distance,
            queue_size=rospy.get_param("MESSAGE_QUEUE_SIZE"))
        self.relative_velocity_subscriber = rospy.Subscriber(
            "vehicle/relative_velocity",
            Range,
            self.__callback_relative_velocity,
            queue_size=rospy.get_param("MESSAGE_QUEUE_SIZE"))

        # TODO: Delete this shit (i.e., rewrite it in the driver)
        self.turn_left_service = rospy.ServiceProxy(
            "vehicle_turn_left", Trigger)
        self.turn_right_service = rospy.ServiceProxy(
            "vehicle_turn_right", Trigger)
        self.is_on_line_service = rospy.ServiceProxy(
            "vehicle_is_on_line", Trigger)
        self.is_not_on_line_service = rospy.ServiceProxy(
            "vehicle_is_not_on_line", Trigger)
        self.is_left_of_line_service = rospy.ServiceProxy(
            "vehicle_is_left_of_line", Trigger)
        self.is_right_of_line_service = rospy.ServiceProxy(
            "vehicle_is_right_of_line", Trigger)
        self.is_slightly_left_of_line_service = rospy.ServiceProxy(
            "vehicle_is_slightly_left_of_line", Trigger)
        self.is_slightly_right_of_line_service = rospy.ServiceProxy(
            "vehicle_is_slightly_right_of_line", Trigger)

        rospy.Timer(rospy.Duration(
            rospy.get_param("PLATOONING_PERIOD")),
            self.__perform_step)

    def __callback_relative_velocity(self, data: Range):
        self.__current_relative_velocity = data.range

    def __callback_distance(self, data: Range):
        self.__current_distance = data.range

    def __perform_step(self, event):
        """
        Performs a single step of the platooning algorithm.
        """
        min_dist = rospy.get_param("MINIMUM_DISTANCE_THRESHOLD")

        if self.__current_distance < min_dist:
            self.speed_publisher.publish(0)
        else:
            self.speed_publisher.publish(rospy.get_param("MAX_SPEED_MOTOR"))

        # TODO: Do line following

    def stop(self):
        """
        Stops the platooning node.
        """
        self.distance_subscriber.unregister()
        self.relative_velocity_subscriber.unregister()

if __name__ == "__main__":
    rospy.init_node("platooning_node")
    controller = PlatooningController()
    rospy.on_shutdown(controller.stop)

    rospy.loginfo("Platooning node started.")
    rospy.spin()
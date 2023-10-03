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

        self.speed_publisher = rospy.Publisher("vehicle/speed", Int32, queue_size=10)
        self.direction_publisher = rospy.Publisher("vehicle/direction", Int32, queue_size=10)

        self.distance_subscriber = rospy.Subscriber("vehicle/distance", Range, queue_size=10)
        self.relative_velocity_subscriber = rospy.Subscriber("vehicle/relative_velocity", Range, queue_size=10)

        self.turn_left_service = rospy.ServiceProxy("vehicle_turn_left", Trigger)
        self.turn_right_service = rospy.ServiceProxy("vehicle_turn_right", Trigger)
        self.is_on_line_service = rospy.ServiceProxy("vehicle_is_on_line", Trigger)
        self.is_not_on_line_service = rospy.ServiceProxy("vehicle_is_not_on_line", Trigger)
        self.is_left_of_line_service = rospy.ServiceProxy("vehicle_is_left_of_line", Trigger)
        self.is_right_of_line_service = rospy.ServiceProxy("vehicle_is_right_of_line", Trigger)
        self.is_slightly_left_of_line_service = rospy.ServiceProxy("vehicle_is_slightly_left_of_line", Trigger)
        self.is_slightly_right_of_line_service = rospy.ServiceProxy("vehicle_is_slightly_right_of_line", Trigger)

    def stop(self):
        """
        Stops the platooning node.
        """
        self.speed_publisher.publish(0)
        self.direction_publisher.publish(0)

        self.distance_subscriber.unregister()
        self.relative_velocity_subscriber.unregister()

if __name__ == "__main__":
    rospy.init_node("platooning_node")
    controller = PlatooningController()
    rospy.on_shutdown(controller.stop)

    rospy.loginfo("Platooning node started.")
    rospy.spin()
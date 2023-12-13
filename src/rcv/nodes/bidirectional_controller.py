#!/usr/bin/env python3
import time
import rospy
import numpy as np

from sensor_msgs.msg import Range
from std_msgs.msg import Int32, Float32, Bool

from controller.state_space import BidirectionalStateSpace

VEHICLE_ID_LEADER = "vehicle_0"

class AdjacentVehicle:
    def __init__(self, vehicle_id):
        self.__id = vehicle_id
        self.__message_queue_size = rospy.get_param("MESSAGE_QUEUE_SIZE")
        self.__current_velocity = 0
        self.__current_distance = 0

        self.__distance_last_update = -1
        self.__velocity_last_update = -1

        self.__velocity_subscriber = rospy.Subscriber(
            f"/{self.__id}/velocity",
            Float32,
            self.__callback_velocity,
            queue_size=self.__message_queue_size)

        if not self.is_leader():
            self.__distance_subscriber = rospy.Subscriber(
                f"/{self.__id}/distance",
                Range,
                self.__callback_distance,
                queue_size=self.__message_queue_size)

    def __callback_velocity(self, msg: Float32):
        self.__current_velocity = msg.data
        self.__velocity_last_update = time.time_ns()

    def __callback_distance(self, msg: Range):
        self.__current_distance = msg.range
        self.__distance_last_update = time.time_ns()

    @property
    def velocity(self):
        return self.__current_velocity

    @property
    def distance(self):
        return self.__current_distance

    def has_updates_since(self, period):
        """
        Checks if the velocity and distance has been updated within
        'period'. E.g. if period is 0.1, this will return True if both
        velocity and distance has received new values in the past 0.1s.
        """
        now = time.time_ns()
        distance_updated = ((now-self.__distance_last_update) / 10**9) <= period
        velocity_updated = ((now-self.__velocity_last_update) / 10**9) <= period

        # No distance from leader
        if self.is_leader():
            return velocity_updated

        return distance_updated and velocity_updated

    def is_leader(self):
        return self.__id == VEHICLE_ID_LEADER

class BidirectionalController:
    def __init__(self):
        self.__id = rospy.get_param("VEHICLE_ID")
        self.__period = rospy.get_param("BIDIRECTIONAL_CONTROL_PERIOD")

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
        self.__current_distance = 0
        self.__current_velocity = 0
        self.__current_leader_velocity = 0

        (
            vehicle_leader,
            vehicle_in_front,
            vehicle_behind
        ) = self.__get_adjacent_vehicles()

        self.__vehicle_leader = vehicle_leader
        self.__vehicle_behind = vehicle_behind
        self.__vehicle_in_front = vehicle_in_front

        self.__state_space = BidirectionalStateSpace()
        self.__pwm_mapper = self.__create_velocity_pwm_mapper(
            rospy.get_param("VELOCITY_PWM_MAP_FILE_PATH"),
            rospy.get_param("VELOCITY_PWM_MAP_POLYFIT_DEGREE"))

        self.__pwm_publisher = rospy.Publisher(
            f"{self.__id}/pwm",
            Int32,
            queue_size=self.__message_queue_size)

        self.__control_publisher = rospy.Publisher(
            f"{self.__id}/control",
            Float32,
            queue_size=self.__message_queue_size)

        self.__distance_subscriber = rospy.Subscriber(
            f"{self.__id}/distance",
            Range,
            self.__callback_distance,
            queue_size=self.__message_queue_size)

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

        self.__has_target_subscriber = rospy.Subscriber(
            f"{self.__id}/has_target",
            Bool,
            self.__callback_has_target,
            queue_size=self.__message_queue_size)

        rospy.Timer(rospy.Duration(self.__period), self.__perform_step)

    def __get_adjacent_vehicles(self):
        """
        Gets the vehicle id of the vehicle in front and behind.
        """
        self_num = self.__id[self.__id.index("_")+1:]
        vehicle_id_in_front = f"vehicle_{self_num-1}"
        vehicle_id_behind = f"vehicle_{self_num-1}"

        vehicle_leader = None
        vehicle_behind = None
        vehicle_in_front = None

        topics = rospy.get_published_topics()

        if f"/{vehicle_id_in_front}/velocity" in topics:
            vehicle_in_front = AdjacentVehicle(vehicle_id_in_front)

        if f"/{vehicle_id_behind}/velocity" in topics:
            vehicle_behind = AdjacentVehicle(vehicle_id_behind)

        if vehicle_in_front.is_leader():
            vehicle_leader = vehicle_in_front
        elif self.__id != VEHICLE_ID_LEADER:
            vehicle_leader = AdjacentVehicle(VEHICLE_ID_LEADER)
        else:
            rospy.signal_shutdown(
                "The leader vehicle can not run bidirectional controller")

        return (vehicle_leader, vehicle_in_front, vehicle_behind)

    def __create_velocity_pwm_mapper(self, map_file, polyfit_deg):
        data = np.genfromtxt(
            map_file,
            delimiter=",",
            skip_header=1,
            names=["pwm", "velocity"])
        z = np.polyfit(data["velocity"], data["pwm"], polyfit_deg)
        return np.poly1d(z)

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

        # Only apply controller if we have an object to follow,
        # and we have enabled the flag in the launch file.
        new_pwm = self.__idle
        if self.__has_target or not self.__stop_vehicle_if_no_target:
            desired_velocity = self.__state_space.update()
            # TODO: Is this the desired velocity, or the change?
            new_pwm = int(min(
                max(self.__pwm_mapper(desired_velocity), self.__idle),
                self.__max_forward))

        self.__pwm_publisher.publish(new_pwm)
        self.__control_publisher.publish(desired_velocity)

    def stop(self):
        self.__pwm_publisher.publish(self.__idle)

if __name__ == "__main__":
    rospy.init_node("pid_controller_node", anonymous=True)
    controller = PIDController()
    rospy.on_shutdown(controller.stop)

    rospy.loginfo("PID controller node started.")
    rospy.spin()

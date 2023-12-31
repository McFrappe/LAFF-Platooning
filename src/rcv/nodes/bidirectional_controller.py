#!/usr/bin/env python3
import time
import rospy
import numpy as np

from sensor_msgs.msg import Range
from std_msgs.msg import Int32, Float32, Bool

from rcv.velocity_mapper import VelocityMapper
from controller.state_space import BidirectionalStateSpace, VehicleDynamics


class AdjacentVehicle:
    def __init__(self, vehicle_id):
        self.__id = vehicle_id
        self.__id_leader = rospy.get_param("VEHICLE_ID_LEADER")
        self.__message_queue_size = rospy.get_param("MESSAGE_QUEUE_SIZE")
        self.__current_velocity = 0
        self.__current_distance = 0

        self.__distance_last_update = -1
        self.__velocity_last_update = -1
        self.__subscribe = self.__should_subscribe()

        if self.__subscribe:
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

    def __should_subscribe(self):
        topics = [x[0] for x in rospy.get_published_topics()]
        return f"/{self.__id}/velocity" in topics

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

    @property
    def subscribed(self):
        return self.__subscribe

    def has_updates_since(self, period):
        """
        Checks if the velocity and distance has been updated within
        'period'. E.g. if period is 0.1, this will return True if both
        velocity and distance has received new values in the past 0.1s.
        """
        if (
            self.__distance_last_update == -1 or
            self.__velocity_last_update == -1
        ):
            return False

        now = time.time_ns()
        distance_updated = ((now-self.__distance_last_update) / 10**9) <= period
        velocity_updated = ((now-self.__velocity_last_update) / 10**9) <= period

        # No distance from leader
        if self.is_leader():
            return velocity_updated

        return distance_updated and velocity_updated

    def is_leader(self):
        return self.__id == self.__id_leader

class BidirectionalController:
    def __init__(self):
        self.__id = rospy.get_param("VEHICLE_ID")
        self.__id_leader = rospy.get_param("VEHICLE_ID_LEADER")
        self.__order = int(self.__id[self.__id.index("_")+1:])
        self.__period = rospy.get_param("BISS_CONTROL_PERIOD")
        self.__initialize_wait_time_s = rospy.get_param(
            "BISS_INITIALIZE_WAIT_TIME_S")

        self.__debug_mode = rospy.get_param("BISS_DEBUG_MODE")
        self.__debug_print_period = rospy.get_param("BISS_DEBUG_PRINT_PERIOD")

        self.__margin_in_m = rospy.get_param("BISS_MARGIN_M")
        self.__message_queue_size = rospy.get_param("MESSAGE_QUEUE_SIZE")
        self.__stop_vehicle_if_no_target = rospy.get_param(
            "VEHICLE_STOP_IF_NO_OBJECT_VISIBLE")

        self.__min_forward = rospy.get_param("MIN_FORWARD_MOTOR")
        self.__max_forward = rospy.get_param("MAX_FORWARD_MOTOR")
        self.__idle = rospy.get_param("IDLE_MOTOR")

        self.__initialized = False
        self.__has_target = False
        self.__esc_calibrated = False
        self.__current_distance = 0
        self.__current_velocity = 0
        self.__missed_updates = 0

        # Set on initialize
        self.__total_vehicles = 0
        self.__state_space = None
        self.__vehicle_leader = None
        self.__vehicle_in_front = None
        self.__vehicle_behind = None

        self.__velocity_mapper = VelocityMapper(
            rospy.get_param("VELOCITY_PWM_MAP_PATH"),
            rospy.get_param("VELOCITY_PWM_MAP_POLYFIT_DEGREE"),
            self.__idle,
            self.__min_forward,
            self.__max_forward)

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

        rospy.Timer(
            rospy.Duration(self.__initialize_wait_time_s),
            self.__initialize,
            oneshot=True
        )
        rospy.Timer(rospy.Duration(self.__period), self.__perform_step)

        if self.__debug_mode:
            rospy.Timer(
                rospy.Duration(self.__debug_print_period), self.__debug_print)

    def __get_vehicles_in_platoon(self):
        """
        Gets the vehicle  of the vehicle in front and behind.
        """
        if self.__order == 0:
            rospy.signal_shutdown(
                "The leader vehicle can not run bidirectional controller")
            return

        topics = rospy.get_published_topics()
        vehicle_id_in_front = f"vehicle_{self.__order-1}"
        vehicle_id_behind = f"vehicle_{self.__order+1}"
        vehicle_in_front = AdjacentVehicle(vehicle_id_in_front)
        vehicle_behind = AdjacentVehicle(vehicle_id_behind)

        if vehicle_in_front.is_leader():
            vehicle_leader = vehicle_in_front
        else:
            vehicle_leader = AdjacentVehicle(self.__id_leader)

        # Myself and a leader vehicle
        total_vehicles = self.__order + 1
        for i in range(total_vehicles, 25):
            if f"/vehicle_{i}/velocity" in topics:
                total_vehicles += 1
            else:
                break

        return (
            total_vehicles,
            vehicle_leader,
            vehicle_in_front,
            vehicle_behind
        )

    def __initialize(self, event):
        """
        Wait for some time before attempting to find other vehicles
        in the platoon. This is because we need to ensure that all
        vehicles are up and running before checking which topics
        (vehicles) are available.

        A better solution might be to do this manually via the
        orchestrator utility once we see that everything has started.
        """
        (
            self.__total_vehicles,
            self.__vehicle_leader,
            self.__vehicle_in_front,
            self.__vehicle_behind
        ) = self.__get_vehicles_in_platoon()

        self.__state_space = BidirectionalStateSpace(
            self.__order,
            self.__total_vehicles,
            self.__margin_in_m,
            self.__period,
            VehicleDynamics(
                rospy.get_param("VELOCITY_MAX"),
                rospy.get_param("BISS_MAX_ACCELERATION"),
                rospy.get_param("BISS_MAX_DECELERATION"),
                rospy.get_param("BISS_VEHICLE_MASS"),
                rospy.get_param("BISS_VEHICLE_LENGTH"),
                rospy.get_param("BISS_SS_HP"),
                np.array([
                    rospy.get_param("BISS_SS_R_0"),
                    rospy.get_param("BISS_SS_R_1"),
                    rospy.get_param("BISS_SS_R_2")
                ]),
                np.array([
                    rospy.get_param("BISS_SS_A_0"),
                    rospy.get_param("BISS_SS_A_1"),
                    rospy.get_param("BISS_SS_A_2")
                ])
            )
        )
        self.__initialized = True

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

    def __callback_has_target(self, msg: Bool):
        self.__has_target = msg.data

    def __get_missed_update(self, vehicle):
        # Dummy vehicle, no data being transmitted from it.
        if not vehicle.subscribed:
            return 0

        return 0 if vehicle.has_updates_since(self.__period) else 1

    def __min_distance(self):
        speed_in_m_per_s = self.__current_velocity / 3.6
        return speed_in_m_per_s * self.__period * 2 + self.__margin_in_m

    def __perform_step(self, event):
        if not self.__esc_calibrated or not self.__initialized:
            return

        if self.__debug_mode:
            self.__missed_updates += self.__get_missed_update(
                self.__vehicle_in_front)
            self.__missed_updates += self.__get_missed_update(
                self.__vehicle_behind)
            if self.__vehicle_in_front != self.__vehicle_leader:
                self.__missed_updates += self.__get_missed_update(
                    self.__vehicle_leader)

        if not self.__has_target and self.__stop_vehicle_if_no_target:
            # Only apply controller if we have an object to follow,
            # and we have enabled the flag in the launch file.
            new_pwm = self.__idle
            desired_velocity = 0
        else:
            desired_velocity = self.__state_space.update(
                self.__current_distance,
                self.__vehicle_in_front.distance,
                self.__vehicle_behind.distance,
                self.__current_velocity,
                self.__vehicle_leader.velocity,
                self.__vehicle_in_front.velocity,
                self.__vehicle_behind.velocity,
            )
            new_pwm = self.__velocity_mapper.to_pwm(desired_velocity)

        self.__pwm_publisher.publish(new_pwm)
        self.__control_publisher.publish(desired_velocity)

    def __debug_print(self, event):
        if not self.__initialized:
            return

        msg = "Bidirectional controller\n"
        msg += f"Velocity self: {self.__current_velocity}\n"
        msg += f"Velocity leader: {self.__vehicle_leader.velocity}\n"
        msg += f"Velocity in front: {self.__vehicle_in_front.velocity}\n"
        msg += f"Velocity behind: {self.__vehicle_behind.velocity}\n"
        msg += f"Distance self: {self.__current_distance}\n"
        msg += f"Distance in front: {self.__vehicle_in_front.distance}\n"
        msg += f"Distance behind: {self.__vehicle_behind.distance}\n"
        msg += f"Missed updates: {self.__missed_updates}\n"
        rospy.loginfo(msg)

    def stop(self):
        self.__pwm_publisher.publish(self.__idle)

if __name__ == "__main__":
    rospy.init_node("bidirectional_controller_node", anonymous=True)
    controller = BidirectionalController()
    rospy.on_shutdown(controller.stop)

    rospy.loginfo("Bidirectional controller node started.")
    rospy.spin()

#!/usr/bin/env python3
import rospy

from std_msgs.msg import Int32
from std_srvs.srv import Trigger

import joyit.constants as constants
from joyit.L298N_driver import L298NDriver, L298NPinConfig

class MovementController:
    def __init__(self):
        left_pin_config = L298NPinConfig(
            ena=constants.GPIO13_PWM1,
            in1=constants.GPIO2,
            in2=constants.GPIO3,
            in3=constants.GPIO4,
            in4=constants.GPIO14,
            enb=constants.GPIO19_PWM1
        )
        right_pin_config = L298NPinConfig(
            ena=constants.GPIO12_PWM0,
            in1=constants.GPIO17,
            in2=constants.GPIO27,
            in3=constants.GPIO22,
            in4=constants.GPIO23,
            enb=constants.GPIO18_PWM0
        )

        self.left_driver = L298NDriver(
            name="Left",
            config=left_pin_config
        )
        self.right_driver = L298NDriver(
            name="Right",
            config=right_pin_config
        )

        self.speed = 0
        self.setup_service()

    def setup_service(self):
        rospy.Subscriber("vehicle_speed", Int32, self.callback_speed)
        rospy.Subscriber("vehicle_direction", Int32, self.callback_direction)

        rospy.Service("vehicle_turn_left", Trigger, self.turn_left)
        rospy.Service("vehicle_turn_right", Trigger, self.turn_right)

    def callback_speed(self, msg: Int32):
        self.speed = msg.data
        self.left_driver.set_speed(self.speed)
        self.right_driver.set_speed(self.speed)

    def callback_direction(self, msg: Int32):
        self.direction = msg.data
        self.left_driver.set_direction(self.direction)
        self.right_driver.set_direction(self.direction)

    def turn_left(self):
        """
        Turn the robot left. If amount of turn not specified, turn at max speed registered for the motor.
        param: amount: int, between 0 and 100
        """
        self.left_driver.set_speed(0)
        self.right_driver.set_speed(self.speed)
        return {"success": True, "message": "Vehicle is turning left"}

    def turn_right(self):
        """
        Turn the robot right. If amount of turn not specified, turn at max speed registered for the motor.
        param: amount: int, between 0 and 100
        """
        self.left_driver.set_speed(self.speed)
        self.right_driver.set_speed(0)
        return { "success": True, "message": "Vehicle is turning right" }

    def stop(self):
        self.left_driver.set_speed(0)
        self.right_driver.set_speed(0)
        self.left_driver.cleanup()
        self.right_driver.cleanup()

if __name__ == "__main__":
    rospy.init_node("motor_node")
    movement_controller = MovementController()
    rospy.on_shutdown(movement_controller.stop)

    rospy.loginfo("Movement Controller is now intialized.")
    rospy.loginfo(f"Status of {movement_controller.left_driver.name} is {movement_controller.left_driver.get_status()}")
    rospy.loginfo(f"Status of {movement_controller.right_driver.name} is {movement_controller.right_driver.get_status()}")

    rospy.loginfo("Motor node started.")
    rospy.spin()
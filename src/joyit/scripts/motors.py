import rospy

from common import *
from l298n_driver import L298NDriver
from l298n_pin_config import L298NPinConfig

class MovementController:
    def __init__(self):
        left_pin_config = L298NPinConfig(
            ena=GPIO13_PWM1,
            in1=GPIO2,
            in2=GPIO3,
            in3=GPIO4,
            in4=GPIO14,
            enb=GPIO15)
        right_pin_config = L298NPinConfig(
            ena=GPIO12_PWM0,
            in1=GPIO17,
            in2=GPIO27,
            in3=GPIO22,
            in4=GPIO23,
            enb=GPIO24)

        self.left_controller = L298NDriver(name="Left",
                                           config=left_pin_config)
        self.right_controller = L298NDriver(name="Right",
                                            config=right_pin_config)

    def drive(self, absolute_speed: int):
        """
        Drive the robot forward or backward, depending on absolute speed set
        param: absolute_speed: int, between -100 and 100
        """
        self.left_controller.set_speed(absolute_speed)
        self.right_controller.set_speed(absolute_speed)

    def turn_left(self, amount: int):
        self.left_controller.set_speed(0)
        self.right_controller.set_speed_max()

    def turn_right(self, amount: int):
        self.left_controller.set_speed_max()
        self.right_controller.set_speed(0)

if __name__ == "__main__":
    rospy.init_node("motor_node")
    movementController = MovementController()

    rospy.loginfo("Movement Controller is now intialized.")
    rospy.loginfo(f"Status of {movementController.left_controller.name} is {movementController.left_controller.get_status()}")
    rospy.loginfo(f"Status of {movementController.right_controller.name} is {movementController.right_controller.get_status()}")

    rospy.loginfo("Motor driver is now started, ready to get commands.")
    rospy.spin()
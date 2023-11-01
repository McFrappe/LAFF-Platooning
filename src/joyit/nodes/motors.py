#!/usr/bin/env python3
import rospy

from std_msgs.msg import Int32

from joyit.L298N_driver import L298NDriver, L298NPinConfig

class MovementController:
    def __init__(self):
        left_pin_config = L298NPinConfig(
            ena=rospy.get_param("LEFT_MOTOR_ENA"),
            in1=rospy.get_param("LEFT_MOTOR_IN1"),
            in2=rospy.get_param("LEFT_MOTOR_IN2"),
            in3=rospy.get_param("LEFT_MOTOR_IN3"),
            in4=rospy.get_param("LEFT_MOTOR_IN4"),
            enb=rospy.get_param("LEFT_MOTOR_ENB"))
        right_pin_config = L298NPinConfig(
            ena=rospy.get_param("RIGHT_MOTOR_ENA"),
            in1=rospy.get_param("RIGHT_MOTOR_IN1"),
            in2=rospy.get_param("RIGHT_MOTOR_IN2"),
            in3=rospy.get_param("RIGHT_MOTOR_IN3"),
            in4=rospy.get_param("RIGHT_MOTOR_IN4"),
            enb=rospy.get_param("RIGHT_MOTOR_ENB"))

        self.left_driver = L298NDriver(
            name="Left",
            config=left_pin_config)
        self.right_driver = L298NDriver(
            name="Right",
            config=right_pin_config)

        self.speed = 0
        self.setup_service()

    def setup_service(self):
        rospy.Subscriber("vehicle/speed", Int32, self.callback_speed)
        rospy.Subscriber("vehicle/direction", Int32, self.callback_direction)
        rospy.Subscriber("vehicle/turn", Int32, self.callback_turn)

    def callback_speed(self, msg: Int32):
        self.speed = msg.data
        self.left_driver.set_speed(self.speed)
        self.right_driver.set_speed(self.speed)

    def callback_direction(self, msg: Int32):
        self.direction = msg.data
        self.left_driver.set_direction(self.direction)
        self.right_driver.set_direction(self.direction)

    def callback_turn(self, msg: Int32):
        if msg.data == rospy.get_param("TURN_LEFT"):
            self.turn_left()
        elif msg.data == rospy.get_param("TURN_RIGHT"):
            self.turn_right()
        else:
            self.no_turn()

    def no_turn(self):
        """
        Stop turning and go forward/backwards.
        """
        rospy.loginfo("Stop turning")
        self.left_driver.set_speed(self.speed)
        self.right_driver.set_speed(self.speed)

    def turn_left(self):
        """
        Turn the vehicle left.
        """
        rospy.loginfo("Turn left")
        self.left_driver.set_speed(0)
        self.right_driver.set_speed(self.speed)

    def turn_right(self):
        """
        Turn the vehicle right.
        """
        rospy.loginfo("Turn right")
        self.left_driver.set_speed(self.speed)
        self.right_driver.set_speed(0)

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

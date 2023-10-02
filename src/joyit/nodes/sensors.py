#!/usr/bin/env python3
import rospy

from std_srvs.srv import Trigger

import joyit.constants as constants
from joyit.IR_array_driver import IRArrayDriver

class SensorController:
    def __init__(self):
        self.ir_controller = IRArrayDriver(
            constants.GPIO10, constants.GPIO21, constants.GPIO11)
        self.setup_service()

    def setup_service(self):
        rospy.Service("vehicle_is_on_line", Trigger,
                      self.is_on_line)
        rospy.Service("vehicle_is_not_on_line", Trigger,
                      self.is_not_on_line)
        rospy.Service("vehicle_is_left_of_line", Trigger,
                      self.is_left_of_line)
        rospy.Service("vehicle_is_slightly_left_of_line", Trigger,
                      self.is_left_of_line_slightly)
        rospy.Service("vehicle_is_right_of_line", Trigger,
                      self.is_right_of_line)
        rospy.Service("vehicle_is_slightly_right_of_line", Trigger,
                      self.is_right_of_line_slightly)

    # region Public methods for IR array driver
    def is_on_line(self):
        """
        Check if the vehicle is on the line.
        """
        left, middle, right = self.ir_controller.get_all_values()
        if middle == 1 and left == 1 and right == 1:
            status = True
        else:
            status = False

        return {"success": status, "message": "Vehicle is on line"}

    def is_not_on_line(self):
        """
        Check if the vehicle is not on the line.
        """
        left, middle, right = self.ir_controller.get_all_values()
        if middle == 0 and left == 0 and right == 0:
            status = True
        else:
            status = False

        return {"success": status, "message": "Vehicle is not on line"}

    def is_left_of_line(self):
        """
        Check if the vehicle is left of the line.
        """
        left, middle, right = self.ir_controller.get_all_values()
        if left == 0 and middle == 0 and right == 1:
            status = True
        else:
            status = False

        return {"success": status, "message": "Vehicle is left of line"}

    def is_right_of_line(self):
        """
        Check if the vehicle is right of the line.
        """
        left, middle, right = self.ir_controller.get_all_values()
        if left == 1 and middle == 0 and right == 0:
            status = True
        else:
            status = False

        return {"success": status, "message": "Vehicle is right of line"}

    def is_left_of_line_slightly(self):
        """
        check if the vehicle is slightly left of the line.
        """
        left, middle, right = self.ir_controller.get_all_values()
        if left == 0 and middle == 1 and right == 1:
            status = True
        else:
            status = False

        return {"success": status, "message": "Vehicle is slightly left of line"}

    def is_right_of_line_slightly(self):
        """
        check if the vehicle is slightly right of the line.
        """
        left, middle, right = self.ir_controller.get_all_values()
        if left == 1 and middle == 1 and right == 0:
            status = True
        else:
            status = False

        return {"success": status, "message": "Vehicle is slightly right of line"}

    # endregion


if __name__ == "__main__":
    rospy.init_node("sensor_node")
    sensor_controller = SensorController()
    rospy.on_shutdown(sensor_controller.ir_controller.cleanup)

    rospy.loginfo("Sensor Controller started.")
    rospy.spin()

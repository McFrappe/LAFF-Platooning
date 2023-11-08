cmake_minimum_required(VERSION 3.0.2)
project(common)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  sensor_msgs
)

catkin_python_setup()
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  nodes/line_follower.py
  nodes/distance.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
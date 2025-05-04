cmake_minimum_required(VERSION 3.0.2)
project(solve_maze)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf
  tf2_ros
  cv_bridge
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS scripts/mapper.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


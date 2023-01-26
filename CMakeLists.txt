cmake_minimum_required(VERSION 3.0.2)
project(hera_description)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
  tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

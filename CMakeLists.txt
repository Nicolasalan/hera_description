cmake_minimum_required(VERSION 2.8.3)
project(hera_description)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)
find_package(gazebo REQUIRED)


catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
	${GAZEBO_INCLUDE_DIRS}
)

#!/bin/bash
export ROS_DISTRO=noetic

sudo apt-get update

# Install ROS
apt-get install -y ros-$ROS_DISTRO-gazebo-ros 
apt-get install -y ros-$ROS_DISTRO-xacro 

# Install Simulation
apt-get install -y ros-$ROS_DISTRO-robot-state-controller
apt-get install -y ros-$ROS_DISTRO-robot-state-publisher
apt-get install -y ros-$ROS_DISTRO-joint-state-controller
apt-get install -y ros-$ROS_DISTRO-joint-state-publisher

# Install Drivers
apt-get install -y ros-$ROS_DISTRO-driver-base
apt-get install -y ros-$ROS_DISTRO-rosserial-arduino
apt-get install -y ros-$ROS_DISTRO-rosserial_python
apt-get install -y ros-$ROS_DISTRO-rplidar-ros

# Install Teleop
apt-get install -y ros-$ROS_DISTRO-joy
apt-get install -y ros-$ROS_DISTRO-teleop-twist-keyboard

# Install Manipulator
apt-get install -y ros-$ROS_DISTRO-moveit-commander
apt-get install -y ros-$ROS_DISTRO-dynamixel-workbench-msgs
apt-get install -y ros-$ROS_DISTRO-dynamixel-workbench-controllers

# install camera
apt-get install -y ros-$ROS_DISTRO-usb-cam
apt-get install -y ros-$ROS_DISTRO-freenect-stack
apt-get install -y libfreenect-dev

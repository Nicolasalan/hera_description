#!/bin/bash
# Source ROS and Catkin workspaces
source /opt/ros/noetic/setup.bash
source /usr/share/gazebo-11/setup.sh

# Set environment variables
export GAZEBO_MODEL_PATH=/ws_hera/src/aws-robomaker-small-house-world/models

export GAZEBO_MASTER_IP=$(sudo docker inspect --format '{{ .NetworkSettings.IPAddress }}')
export GAZEBO_MASTER_URI=$GAZEBO_MASTER_IP:11345

# Execute the command passed into this entrypoint
cd ~/gzweb && npm start
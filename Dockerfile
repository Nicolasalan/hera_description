# Ubuntu 20.04 image with NVIDIA CUDA + OpenGL and ROS Noetic
FROM nvidia/cuda:11.4.2-base-ubuntu20.04

# Install basic apt packages
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y locales lsb-release
RUN dpkg-reconfigure locales

# Install ROS Noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update \
 && apt-get install -y --no-install-recommends ros-noetic-desktop-full
RUN apt-get install -y --no-install-recommends python3-rosdep
RUN rosdep init \
 && rosdep fix-permissions \
 && rosdep update
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Change the default shell to Bash
SHELL [ "/bin/bash" , "-c" ]

# Setup minimal
RUN apt-get update

# Install dependencies
RUN apt-get install -q -y --no-install-recommends \
  build-essential \
  apt-utils \
  cmake \
  g++ \
  git \
  gnupg gnupg1 gnupg2 \
  libcanberra-gtk* \
  python3-catkin-tools \
  libcanberra-gtk* \
  python3-catkin-tools \
  python3-pip \
  python3-tk \
  python3-yaml \
  python3-dev \
  python3-numpy \
  python3-rosinstall \
  python3-catkin-pkg \
  python3-rosdistro \
  python3-rospkg \
  wget \
  curl \
  vim \
  xterm \
  npm

# depenpencies gzweb
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    build-essential \
    imagemagick \
    libboost-all-dev \
    libgts-dev \
    libjansson-dev \
    libtinyxml-dev \
    mercurial \
    nodejs \
    pkg-config \
    psmisc \
    xvfb

# ROS packages
RUN apt-get install -y --no-install-recommends \
  ros-noetic-py-trees-ros \
  ros-noetic-py-trees \
  ros-noetic-smach-ros \
  ros-noetic-roswww 

# Install dependencies ros
RUN apt-get update && apt-get install -y ros-noetic-ros-controllers \
 && apt-get install -y ros-noetic-joint-state-controller \
 && apt-get install -y ros-noetic-joint-state-publisher \
 && apt-get install -y ros-noetic-robot-state-publisher \
 && apt-get install -y ros-noetic-robot-state-controller \
 && apt-get install -y ros-noetic-xacro \ 
 && apt-get install -y ros-noetic-smach-ros \
 && apt-get install -y ros-noetic-gazebo-ros \
 && apt-get install -y ros-noetic-gazebo-ros-control \
 && apt-get install -y ros-noetic-rplidar-ros \
 && apt-get install -y ros-noetic-driver-base \
 && apt-get install -y ros-noetic-rosserial-arduino

# Gzweb 
RUN apt-get clean

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys D2486D2DD83DB69272AFE98867170598AF249743

# setup sources.list
RUN . /etc/os-release \
    && echo "deb http://packages.osrfoundation.org/gazebo/$ID-stable `lsb_release -sc` main" > /etc/apt/sources.list.d/gazebo-latest.list

RUN apt-get install -y libgazebo11 gazebo11

#install gazebo packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgazebo11-dev=11.12.0-1* 

# clone gzweb
ENV GZWEB_WS /root/gzweb
RUN git clone -b master https://github.com/osrf/gzweb $GZWEB_WS

# setup environment
EXPOSE 8080
EXPOSE 7681

# Rosdep
RUN apt-get install python3-rosdep \
 && rm /etc/ros/rosdep/sources.list.d/20-default.list \
 && sudo rosdep init \
 && rosdep update 

RUN apt install -y locales screen
RUN locale-gen en_GB.UTF-8 && locale-gen en_US.UTF-8

RUN mkdir -p /root/gzweb/http/client/assets

# create a catkin workspace
RUN mkdir -p /ws_hera/src \
 && cd /ws_hera/src \
 && source /opt/ros/noetic/setup.bash \
 && catkin_init_workspace \
 && git clone -b main https://github.com/dheera/rosboard.git \
 && git clone -b ros1 https://github.com/aws-robotics/aws-robomaker-small-house-world

# Copy the source files
COPY . /ws_hera/src
COPY install_dependencies.sh /ws_hera/install_dependencies.sh
COPY entrypoint.sh /ws_hera/entrypoint.sh

# Set the working directory
WORKDIR /ws_hera

# Build the Catkin workspace
RUN cd /ws_hera \
 && source /opt/ros/noetic/setup.bash \
 && rosdep install -y --from-paths src --ignore-src \
 && catkin build

# Setup bashrc
RUN echo "source /ws_hera/devel/setup.bash" >> ~/.bashrc \
 && echo "export ROS_MASTER_URI=http://localhost:11311" >> ~/.bashrc \
 && echo "export ROS_HOSTNAME=localhost" >> ~/.bashrc

# Install python dependencies
RUN cd /ws_hera && ./install_dependencies.sh

# Remove display warnings
RUN mkdir /tmp/runtime-root
ENV XDG_RUNTIME_DIR "/tmp/runtime-root"
ENV NO_AT_BRIDGE 1

# command to run on container start
ENTRYPOINT [ "/ws_hera/entrypoint.sh" ]
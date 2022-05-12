#!/usr/bin/env bash

set -euxo pipefail

sudo apt install -y python3-pip python3-dev python3-numpy python3-empy \
	python3-catkin-pkg-modules python3-rospkg-modules

python3 -m pip install --upgrade pip pyyaml
python3 -m pip install flask gevent numpy requests pynput ipympl pandas matplotlib rospkg
# use catkin build instead of catkin_make
python3 -m pip install catkin_tools

SRC_DIR=~/catkin_ws/src
ROS_DISTRO=$(rosversion -d)
mkdir -vp SRC_DIR
cd SRC_DIR
[ ! -d $SRC_DIR/penguinpi_description ] && git clone https://bitbucket.org/cirrusrobotics/penguinpi_description.git
[ ! -d $SRC_DIR/penguinpi_gazebo ] && git clone https://bitbucket.org/cirrusrobotics/penguinpi_gazebo.git
[ ! -d $SRC_DIR/vision_opencv ] && git clone -b "$ROS_DISTRO" https://github.com/ros-perception/vision_opencv.git

cd ~/catkin_ws
# make sure you run source /opt/ros/$ROS_DISTRO/setup.[bash or zsh] outside this script!!
# CURR_SHELL=$(echo "$SHELL" | sed 's/.*\///')
# source /opt/ros/$ROS_DISTRO/setup.$CURR_SHELL
catkin config -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
catkin build

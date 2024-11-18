#!/bin/bash

sudo apt-get install python3-rosdep -y

source /opt/ros/foxy/setup.bash

sudo rosdep init

rosdep update

rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

rosdep update

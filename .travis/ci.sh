#!/bin/bash

# Software License Agreement - BSD License
#
# Inspired by MoveIt! CI https://github.com/ros-planning/moveit_ci/blob/master/travis.sh
# Author:  Salvo Virga

apt-get -qq update
apt-get -qq dist-upgrade
rosdep update 
apt-get -y install python-catkin-tools
cd /root/iiwa_stack_ws 
rosdep install --from-paths src --ignore-src -r -y
catkin build --no-status --summarize
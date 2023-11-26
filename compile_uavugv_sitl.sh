#!/bin/sh

# Exit immediately if a command exits with a non-zero status.
set -e

# basic messages
catkin_make --source uavros_msgs --build build/uavros_msgs

# gazebo simulation
catkin_make --source uavros_simulation/uavros_gazebo --build build/uavros_gazebo

# function
catkin_make --source uavros_simulation/uavros_uavugv_sitl --build build/uavros_uavugv_sitl

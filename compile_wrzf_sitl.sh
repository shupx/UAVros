#!/bin/sh

# Exit immediately if a command exits with a non-zero status.
set -e

# basic messages
catkin_make --source uavros_msgs --build build/uavros_msgs

# gazebo simulation
catkin_make --source uavros_simulation/uavros_gazebo --build build/uavros_gazebo
catkin_make --source dependenct_packages/ugv_simulator --build build/ugv_simulator

# function
catkin_make --source uavros_simulation/uavros_wrzf_sitl --build build/uavros_wrzf_sitl



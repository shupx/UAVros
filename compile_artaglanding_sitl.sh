#!/bin/sh

# Exit immediately if a command exits with a non-zero status.
set -e

# basic messages
catkin_make --source uavros_msgs --build build/uavros_msgs

# gazebo simulation
catkin_make --source uavros_simulation/uavros_gazebo --build build/uavros_gazebo

# function
catkin_make --source dependenct_packages/ar_track_alvar --build build/ar_track_alvar
catkin_make --source uavros_simulation/ARtagLanding_sitl --build build/uavros_artaglanding_sitl

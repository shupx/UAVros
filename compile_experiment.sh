#!/bin/sh

# Exit immediately if a command exits with a non-zero status.
set -e

# basic messages
catkin_make --source uavros_msgs --build build/uavros_msgs

# function
catkin_make --source dependenct_packages/ar_track_alvar --build build/ar_track_alvar
catkin_make --source uavros_experiment/ARtagLanding --build build/uavros_artaglanding

# function
catkin_make --source uavros_experiment/uavros_uavugv --build build/uavros_uavugv

# function
catkin_make --source car_video --build build/car_video
catkin_make --source uavros_simulation/uavros_wrzf_sitl --build build/uavros_wrzf_sitl

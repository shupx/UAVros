#!/bin/sh

# Exit immediately if a command exits with a non-zero status.
set -e

# basic messages
catkin_make --source uavros_msgs --build build/uavros_msgs

# gazebo simulation
catkin_make --source uavros_simulation/uavros_gazebo --build build/uavros_gazebo

# function
catkin_make --source uavros_simulation/kcffollow_simulation --build build/uavros_kcffollow_simulation

# function
catkin_make --source dependenct_packages/ar_track_alvar --build build/ar_track_alvar
catkin_make --source uavros_simulation/ARtagLanding_sitl --build build/uavros_artaglanding_sitl

# function
catkin_make --source uavros_simulation/uavros_uavugv_sitl --build build/uavros_uavugv_sitl

# function
catkin_make --source dependenct_packages/ugv_simulator --build build/ugv_simulator
catkin_make --source uavros_simulation/uavros_wrzf_sitl --build build/uavros_wrzf_sitl

# function
# catkin_make --source dependenct_packages/ugv_simulator --build build/ugv_simulator
catkin_make --source dependenct_packages/amov_gimbal_sdk_ros --build build/amov_gimbal_sdk_ros
catkin_make --source uavros_simulation/uavros_multi_gimbal_sitl --build build/uavros_multi_gimbal_sitl


#!/bin/sh

# Exit immediately if a command exits with a non-zero status.
set -e

cd ~

sudo apt install xmlstarlet # generate multiple UAV sdf model

### install mavros
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras # for ubuntu 20
wget https://gitee.com/shu-peixuan/px4mocap/raw/85b46df9912338f775949903841160c873af4a1d/ROS-install-command/install_geographiclib_datasets.sh
sudo chmod a+x ./install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh # this step takes some time
rm install_geographiclib_datasets.sh

### install some ros package
sudo apt install -y ros-noetic-gazebo-ros # gazebo-ros. may be needed by uavros_gazebo build
sudo apt install -y ros-noetic-gazebo-plugins # in case of camera image lose (lack gazebo plugin)
sudo apt install -y ros-noetic-ackermann-msgs # ackermann UGV control
sudo apt install -y ros-noetic-usb-cam  # usb camera
sudo apt install -y ros-noetic-image-proc # image calibrate
sudo apt install -y ros-noetic-image-pipeline # image compress and process
sudo apt install -y ros-noetic-camera-calibration # camera calibrate
sudo apt install -y ros-noetic-effort-controllers # racer model control
sudo apt install -y ros-noetic-plotjuggler-ros # curve plot and display
# ROS noetic apt does not have ar-track-alvar, compile the local folder instead.
sudo apt install -y ros-melodic-pcl_conversions # compile ar-track-alvar needs pcl_conversions

sudo apt install python-is-python3 # if you use ubuntu20 but with default python2

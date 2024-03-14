#!/bin/sh

# Exit immediately if a command exits with a non-zero status.
set -e

cd ~

sudo apt install xmlstarlet # generate multiple UAV sdf model

### install mavros
sudo apt install ros-melodic-mavros ros-melodic-mavros-extras  # for ubuntu 18
wget https://gitee.com/shu-peixuan/ros-install-command/raw/c9865c748045a0cce0173fcfcb95729784bd31e5/install_geographiclib_datasets.sh
sudo chmod a+x ./install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh # this step takes some time
rm install_geographiclib_datasets.sh

### install some ros package
sudo apt install -y ros-melodic-gazebo-ros # gazebo-ros. may be needed by uavros_gazebo build
sudo apt install -y ros-melodic-gazebo-plugins # in case of camera image lose (lack gazebo plugin)
sudo apt install -y ros-melodic-ackermann-msgs 
sudo apt install -y ros-melodic-usb-cam 
sudo apt install -y ros-melodic-image-proc 
sudo apt install -y ros-melodic-image-pipeline 
sudo apt install -y ros-melodic-camera-calibration 
sudo apt install -y ros-melodic-effort-controllers 
sudo apt install -y ros-melodic-plotjuggler-ros 
sudo apt install -y ros-melodic-ar-track-alvar 

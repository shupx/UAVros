#!/bin/sh

# Exit immediately if a command exits with a non-zero status.
set -e

cd ~

sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
wget https://gitee.com/shu-peixuan/px4mocap/raw/5acc785ab0a220d9fed36c335b81e28d15ec6936/ROS-install-command/ros.key
sudo apt-key add ros.key
rm ros.key
sudo apt-get update --fix-missing
sudo apt install ros-noetic-desktop # for ubuntu 20
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc # for ubuntu 20

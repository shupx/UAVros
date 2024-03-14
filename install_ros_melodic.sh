#!/bin/sh

# Exit immediately if a command exits with a non-zero status.
set -e

cd ~

sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
wget https://gitee.com/shu-peixuan/ros-install-command/raw/c9865c748045a0cce0173fcfcb95729784bd31e5/ros.key
sudo apt-key add ros.key
rm ros.key
sudo apt-get update --fix-missing
sudo apt install ros-melodic-desktop # for ubuntu 18
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc # for ubuntu 18

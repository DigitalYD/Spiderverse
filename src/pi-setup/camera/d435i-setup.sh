#!/bin/bash

sudo apt update
sudo apt upgrade -y

sudo apt install -y software-properties-common
sudo apt install -y python3-pip
sudo apt install -y python3-venv

echo 'deb [arch=arm64] https://librealsense.intel.com/Debian/apt-repo jammy main' | sudo tee /etc/apt/sources.list.d/librealsense.list

curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

sudo apt update
sudo apt install -y librealsense2-utils librealsense2-dkms

sudo apt install -y curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y ros-humble-realsense2-*

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
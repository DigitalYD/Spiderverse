#!/bin/bash

# Source the ROS2 setup file
source ./install/setup.bash

# Launch the bilateration node
ros2 launch lidar_udp_receiver bilateration.launch.py
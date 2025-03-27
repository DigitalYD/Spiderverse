#!/bin/bash
# Save as ~/lidar_slam.sh

# Source ROS2 and workspace setup
source /opt/ros/humble/setup.bash
source ~/Documents/Spiderverse/lidar_ws/install/setup.bash

# Define cleanup function to kill all background processes
cleanup() {
    echo "Shutting down all processes..."
    # Kill all child processes
    if [ -n "$LIDAR_PID" ]; then
        kill $LIDAR_PID 2>/dev/null
    fi
    if [ -n "$TF_PID" ]; then
        kill $TF_PID 2>/dev/null
    fi
    if [ -n "$CART_PID" ]; then
        kill $CART_PID 2>/dev/null
    fi
    if [ -n "$GRID_PID" ]; then
        kill $GRID_PID 2>/dev/null
    fi
    exit 0
}

# Setup trap for clean shutdown
trap cleanup SIGINT SIGTERM

echo "Starting LiDAR UDP receiver..."
# Start the LIDAR UDP receiver in the background
ros2 launch lidar_udp_receiver lidar_receiver.launch.py &
LIDAR_PID=$!

# Sleep briefly to ensure the LIDAR node starts up first
sleep 2

echo "Starting TF broadcaster..."
# Start the TF broadcaster for coordinate frames
ros2 run lidar_udp_receiver tf_broadcaster &
TF_PID=$!

sleep 1

echo "Starting Cartographer SLAM..."
# Start Cartographer node
ros2 run cartographer_ros cartographer_node \
    -configuration_directory ~/Documents/Spiderverse/lidar_ws/src/lidar_udp_receiver/config \
    -configuration_basename udp_lidar_cartographer_config.lua &
CART_PID=$!

sleep 1

echo "Starting Occupancy Grid node..."
# Start Occupancy Grid node
ros2 run cartographer_ros occupancy_grid_node \
    -resolution 0.05 \
    -publish_period_sec 1.0 &
GRID_PID=$!

sleep 1

echo "Starting RViz2 for visualization..."
# Run RViz2 with the isolated environment
env -i \
  HOME=$HOME \
  USER=$USER \
  DISPLAY=$DISPLAY \
  XAUTHORITY=$XAUTHORITY \
  DBUS_SESSION_BUS_ADDRESS=$DBUS_SESSION_BUS_ADDRESS \
  XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
  PATH=/usr/bin:/bin:/usr/local/bin \
  LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu \
  bash -c 'source /opt/ros/humble/setup.bash && source ~/Documents/Spiderverse/lidar_ws/install/setup.bash && exec /opt/ros/humble/bin/rviz2 -d ~/Documents/Spiderverse/lidar_ws/src/lidar_udp_receiver/config/slam.rviz'

# When RViz2 is closed, kill all other processes
cleanup
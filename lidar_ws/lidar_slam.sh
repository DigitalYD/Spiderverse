#!/bin/bash
# Source ROS2 and workspace setup
source /opt/ros/humble/setup.bash
source ~/Documents/Spiderverse/lidar_ws/install/setup.bash

# First, run cleanup to ensure no processes are already running
echo "Running cleanup to ensure no existing processes..."
~/Documents/Spiderverse/lidar_ws/cleanup.sh

# Define cleanup function to kill all background processes
cleanup() {
    echo "Shutting down all processes..."
    if [ -n "$LIDAR_PID" ]; then
        kill -9 $LIDAR_PID 2>/dev/null
    fi
    if [ -n "$STATIC_TF_PID" ]; then
        kill -9 $STATIC_TF_PID 2>/dev/null
    fi
    if [ -n "$TRILATERATION_PID" ]; then
        kill -9 $TRILATERATION_PID 2>/dev/null
    fi
    if [ -n "$SLAM_LAUNCH_PID" ]; then
        kill -9 $SLAM_LAUNCH_PID 2>/dev/null
    fi
    
    echo "Running final cleanup to ensure all processes terminated..."
    ~/Documents/Spiderverse/lidar_ws/cleanup.sh
    exit 0
}

# Setup trap for clean shutdown
trap cleanup SIGINT SIGTERM EXIT

CONFIG_DIR=~/Documents/Spiderverse/lidar_ws/src/lidar_udp_receiver/config

echo "Starting LiDAR UDP receiver..."
# Start the LIDAR UDP receiver
ros2 launch lidar_udp_receiver lidar_receiver.launch.py &
LIDAR_PID=$!

# Sleep briefly to ensure the LIDAR node starts up first
sleep 2

# Start the TF broadcaster with parameters for improved motion tracking
echo "Starting TF broadcaster..."
ros2 run lidar_udp_receiver tf_broadcaster --ros-args -p use_odometry:=false -p publish_rate:=30.0 &
STATIC_TF_PID=$!

# Give TF broadcaster time to start
sleep 2

echo "Starting SLAM with Cartographer and Trilateration..."
# Launch Cartographer SLAM with trilateration integrated
ros2 launch lidar_udp_receiver cartographer_slam_launch.py \
    use_trilateration:=true &
SLAM_LAUNCH_PID=$!

# Give Cartographer time to start
sleep 3

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
  bash -c "source /opt/ros/humble/setup.bash && source ~/Documents/Spiderverse/lidar_ws/install/setup.bash && exec /opt/ros/humble/bin/rviz2 -d $CONFIG_DIR/slam.rviz"

# When RViz2 is closed, kill all other processes
# This is done via the EXIT trap now
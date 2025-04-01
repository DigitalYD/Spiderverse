#!/bin/bash
# Cleanup script to ensure all ROS2 nodes are properly terminated

echo "Stopping all Cartographer and ROS2 related processes..."

# Find and kill any cartographer processes
CART_PIDS=$(ps aux | grep cartographer | grep -v grep | awk '{print $2}')
if [ ! -z "$CART_PIDS" ]; then
    echo "Killing Cartographer processes: $CART_PIDS"
    for pid in $CART_PIDS; do
        kill -9 $pid 2>/dev/null
    done
fi

# Find and kill any ROS2 related processes
ROS_PIDS=$(ps aux | grep ros2 | grep -v grep | awk '{print $2}')
if [ ! -z "$ROS_PIDS" ]; then
    echo "Killing ROS2 processes: $ROS_PIDS"
    for pid in $ROS_PIDS; do
        kill -9 $pid 2>/dev/null
    done
fi

# Find and kill any Python nodes (like the TF broadcaster)
PYTHON_PIDS=$(ps aux | grep "lidar_udp_receiver.lidar_udp_receiver\|tf_broadcaster" | grep -v grep | awk '{print $2}')
if [ ! -z "$PYTHON_PIDS" ]; then
    echo "Killing Python ROS2 nodes: $PYTHON_PIDS"
    for pid in $PYTHON_PIDS; do
        kill -9 $pid 2>/dev/null
    done
fi

# Find and kill any rviz2 processes
RVIZ_PIDS=$(ps aux | grep rviz2 | grep -v grep | awk '{print $2}')
if [ ! -z "$RVIZ_PIDS" ]; then
    echo "Killing RViz2 processes: $RVIZ_PIDS"
    for pid in $RVIZ_PIDS; do
        kill -9 $pid 2>/dev/null
    done
fi

echo "Checking for any remaining ROS2 processes..."
ps aux | grep -E "cartographer|ros2|rviz2|lidar_udp_receiver" | grep -v grep

echo "Cleanup complete!"
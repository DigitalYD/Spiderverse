#!/bin/bash
# Script to fix Cartographer configuration and setup

# Source ROS2 setup
source /opt/ros/humble/setup.bash
source ~/Documents/Spiderverse/lidar_ws/install/setup.bash

# First, clean up existing processes
echo "Cleaning up existing processes..."
~/Documents/Spiderverse/lidar_ws/cleanup.sh

# Create a more standard configuration for Cartographer
CONFIG_DIR=~/Documents/Spiderverse/lidar_ws/src/lidar_udp_receiver/config

echo "Creating standard Cartographer configuration..."
cat > $CONFIG_DIR/standard_cartographer_config.lua << 'EOL'
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "lidar_link",
  published_frame = "lidar_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4

TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.0
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6

return options
EOL

echo "Starting a fresh Cartographer session..."

# Start LiDAR UDP receiver
echo "Starting LiDAR UDP receiver..."
ros2 launch lidar_udp_receiver lidar_receiver.launch.py &
LIDAR_PID=$!

sleep 3

# Start TF broadcaster
echo "Starting TF broadcaster..."
ros2 run lidar_udp_receiver tf_broadcaster &
TF_PID=$!

sleep 2

# Start Cartographer with simplified config
echo "Starting Cartographer SLAM..."
ros2 run cartographer_ros cartographer_node \
    -configuration_directory $CONFIG_DIR \
    -configuration_basename standard_cartographer_config.lua &
CART_PID=$!

sleep 3

# Start Occupancy Grid node
echo "Starting Occupancy Grid node..."
ros2 run cartographer_ros occupancy_grid_node \
    -resolution 0.05 \
    -publish_period_sec 1.0 &
GRID_PID=$!

sleep 3

# Check what topics we have
echo "Checking available topics..."
ros2 topic list | sort

echo ""
echo "To save a map from submaps, run: ./submap_to_map.sh in another terminal"
echo "Press Ctrl+C when done mapping"

# Define cleanup function
cleanup() {
    echo "Shutting down all processes..."
    for pid in $LIDAR_PID $TF_PID $CART_PID $GRID_PID; do
        if [ -n "$pid" ]; then
            kill -9 $pid 2>/dev/null
        fi
    done
    
    echo "Running final cleanup..."
    ~/Documents/Spiderverse/lidar_ws/cleanup.sh
    exit 0
}

# Set trap for clean shutdown
trap cleanup SIGINT SIGTERM EXIT

# Wait for Ctrl+C
wait
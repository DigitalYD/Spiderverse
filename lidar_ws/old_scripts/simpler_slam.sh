#!/bin/bash
# Simplified SLAM script with focus on generating usable maps
# This version uses Cartographer but with better defaults for RPLidar

# Source ROS2 setup
source /opt/ros/humble/setup.bash
source ~/Documents/Spiderverse/lidar_ws/install/setup.bash

# First, clean up existing processes
echo "Cleaning up existing processes..."
~/Documents/Spiderverse/lidar_ws/cleanup.sh

# Define cleanup function
cleanup() {
    echo "Shutting down all processes..."
    for pid in ${PIDS[@]}; do
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

# Array to store PIDs
PIDS=()

# Create updated Cartographer config
CONFIG_DIR=~/Documents/Spiderverse/lidar_ws/src/lidar_udp_receiver/config
echo "Creating optimized Cartographer configuration..."

cat > $CONFIG_DIR/simple_cartographer_config.lua << 'EOL'
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
  publish_frame_projected_to_2d = true,
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
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4

-- RPLidar-specific settings
TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 8.0  -- Reduced for better accuracy with RPLidar
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.0

-- Scan matching improvements
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 20.0
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)

-- Motion detection for non-odometry setups
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.3
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = 0.1

-- Map resolution
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 100
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

-- Loop closure settings
POSE_GRAPH.optimize_every_n_nodes = 80
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6
POSE_GRAPH.constraint_builder.sampling_ratio = 0.1

return options
EOL

echo "Starting LiDAR UDP receiver..."
ros2 launch lidar_udp_receiver lidar_receiver.launch.py &
PIDS+=($!)

sleep 2

echo "Starting TF broadcaster..."
ros2 run lidar_udp_receiver tf_broadcaster --ros-args -p use_odometry:=false -p publish_rate:=30.0 &
PIDS+=($!)

sleep 2

echo "Starting Cartographer SLAM..."
ros2 run cartographer_ros cartographer_node \
    -configuration_directory $CONFIG_DIR \
    -configuration_basename simple_cartographer_config.lua &
PIDS+=($!)

sleep 3

echo "Starting Occupancy Grid node..."
ros2 run cartographer_ros occupancy_grid_node \
    -resolution 0.05 \
    -publish_period_sec 1.0 &
PIDS+=($!)

sleep 1

echo "SLAM is now running."
echo "Move your RPLidar around the environment to build a map."
echo ""
echo "Commands:"
echo "  - Press Ctrl+C when done to stop mapping"
echo "  - Run './display_map.sh' to view the latest saved map"
echo "  - Run './save_latest_map.sh' to save the current map"
echo ""

# Wait until user presses Ctrl+C
echo "Press Ctrl+C to stop mapping and exit"
wait
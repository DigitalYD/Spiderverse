include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",  -- Change to imu_link for IMU-based tracking
  published_frame = "lidar_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  use_odometry = true,
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
  odometry_sampling_ratio = 0.1,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.0,  -- Use all IMU data
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4

-- Tune scan matcher for RPLidar
TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 10.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.use_imu_data = true  -- Enable IMU data

-- Critical scan matching parameters - increased to favor LiDAR over UWB
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 40.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 20.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 80.0

-- IMU-specific parameters for improving motion estimation
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.0  -- Adjusted for IMU
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.0     -- Adjusted for IMU

-- Configure for accurate motion detection (most important part!)
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = 0.1

-- Adjust real-time correlative scan matcher for better performance
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)

-- Map resolution and size
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 100
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

-- Global SLAM loop closure parameters - improved for better loop closures
POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimize_every_n_nodes = 30
POSE_GRAPH.constraint_builder.min_score = 0.50
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.55
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 10.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(45.)

-- Increased these values to create more loop closure constraints
POSE_GRAPH.global_sampling_ratio = 0.01
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3

-- Adaptive voxel filter for scan matching
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 10.0

-- IMU settings for 2D SLAM
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.0

-- Additional loop closure optimization parameters
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 50
POSE_GRAPH.optimization_problem.acceleration_weight = 1e1
POSE_GRAPH.optimization_problem.rotation_weight = 1e2
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e4
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1.0e3
POSE_GRAPH.max_num_final_iterations = 200

return options
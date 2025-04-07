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
  publish_frame_projected_to_2d = true,  -- Enable 2D projection
  use_pose_extrapolator = true,
  use_odometry = true,  -- Use odometry - can be from trilateration
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
  odometry_sampling_ratio = 0.1,  -- Reduced from 0.5 to make mapping less dependent on UWB
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4

-- Tune scan matcher for RPLidar
TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 10.0  -- Reduced from 40m to avoid noise at long ranges
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- Critical scan matching parameters - increased to favor LiDAR over UWB
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 40.0  -- Doubled from 20.0 to rely more on LiDAR
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 20.0  -- Doubled from 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 80.0  -- Doubled from 40.0

-- Configure for accurate motion detection (most important part!)
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1  -- Detect small movements
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = 0.1   -- Detect small rotations

-- Adjust real-time correlative scan matcher for better performance
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)

-- Map resolution and size
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 100  -- More scans per submap for better quality
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

-- Global SLAM loop closure parameters - improved for better loop closures
POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimize_every_n_nodes = 30  -- Lower from 90 to run optimization more frequently
POSE_GRAPH.constraint_builder.min_score = 0.50  -- Lower from 0.55 to consider more potential matches
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.55  -- Lower from 0.6
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 10.0  -- Increased from 7.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(45.)  -- Increased from 30°

-- Increased these values to create more loop closure constraints
POSE_GRAPH.global_sampling_ratio = 0.01  -- Increased from 0.003 (more global constraint candidates)
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3  -- Increased from 0.1 (evaluate more constraints)

-- Adaptive voxel filter for scan matching
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 10.0

-- Additional loop closure optimization parameters
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 50  -- Increased iterations
POSE_GRAPH.optimization_problem.acceleration_weight = 1e1  -- Tuned for better convergence
POSE_GRAPH.optimization_problem.rotation_weight = 1e2  -- Added to improve rotation consistency
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e4  -- Weight loop closures higher
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1.0e3  -- Weight loop closures higher
POSE_GRAPH.max_num_final_iterations = 200  -- More iterations in final optimization

return options
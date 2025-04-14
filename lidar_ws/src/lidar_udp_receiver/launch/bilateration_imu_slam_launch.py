import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('lidar_udp_receiver')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    cartographer_config_dir_arg = DeclareLaunchArgument(
        'cartographer_config_dir',
        default_value=os.path.join(pkg_dir, 'config'),
        description='Directory for Cartographer configuration files'
    )
    
    configuration_basename_arg = DeclareLaunchArgument(
        'configuration_basename',
        default_value='imu_cartographer_config.lua',
        description='Basename of the Cartographer configuration file'
    )
    
    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.05',
        description='Resolution of the map (meters per pixel)'
    )
    
    publish_period_sec_arg = DeclareLaunchArgument(
        'publish_period_sec',
        default_value='1.0',
        description='OccupancyGrid publishing period'
    )
    
    # Include the LiDAR receiver launch file
    lidar_receiver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lidar_udp_receiver'),
                'launch',
                'lidar_receiver.launch.py'
            ])
        ])
    )
    
    # Include the IMU receiver launch file
    imu_receiver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lidar_udp_receiver'),
                'launch',
                'imu_receiver.launch.py'
            ])
        ])
    )
    
    # Include the TF broadcaster launch file for coordinate transforms
    tf_broadcaster_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lidar_udp_receiver'),
                'launch',
                'tf_broadcaster.launch.py'
            ])
        ])
    )
    
    # Bilateration node for UWB positioning
    bilateration_node = Node(
        package='lidar_udp_receiver',
        executable='bilateration_node',
        name='bilateration_node',
        output='screen',
        parameters=[{
            'frame_id': 'map',
            'position_topic': 'positioning_pose',
            'odometry_topic': 'bilateration_odom',
            'polling_period_ms': 100,
            # Anchor positions in centimeters
            'anchor1_pos': [310.0, 0.0, 90.0],
            'anchor2_pos': [0.0, 0.0, 90.0],
            # Reference for selecting between ambiguous solutions
            'reference_y': 300.0,
            'prefer_positive_y': False,
            # Uncertainty and filtering parameters
            'position_uncertainty': 0.35,
            'use_moving_average': True,
            'moving_average_window': 5,
            'max_position_jump': 1.0
        }]
    )
    
    # Cartographer node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            '-configuration_directory', LaunchConfiguration('cartographer_config_dir'),
            '-configuration_basename', LaunchConfiguration('configuration_basename')
        ],
        remappings=[
            ('scan', 'scan'),
            ('imu', 'imu'),
            ('odom', 'bilateration_odom')  # Use bilateration odometry
        ]
    )
    
    # Occupancy grid node
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'resolution': LaunchConfiguration('resolution')},
            {'publish_period_sec': LaunchConfiguration('publish_period_sec')}
        ]
    )
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'config', 'slam.rviz')],
        output='screen'
    )
    
    # Return launch description
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        cartographer_config_dir_arg,
        configuration_basename_arg,
        resolution_arg,
        publish_period_sec_arg,
        
        # Launch files
        lidar_receiver_launch,
        imu_receiver_launch,
        tf_broadcaster_launch,
        
        # Nodes
        bilateration_node,
        cartographer_node,
        occupancy_grid_node,
        rviz_node
    ])
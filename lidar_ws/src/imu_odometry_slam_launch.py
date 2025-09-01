#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Define LaunchConfiguration variables
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir')
    configuration_basename = LaunchConfiguration('configuration_basename')
    use_imu = LaunchConfiguration('use_imu')
    
    # Create argument declarations
    cartographer_config_dir_arg = DeclareLaunchArgument(
        'cartographer_config_dir',
        default_value=PathJoinSubstitution([FindPackageShare('lidar_udp_receiver'), 'config']),
        description='Full path to cartographer config file directory'
    )
    
    configuration_basename_arg = DeclareLaunchArgument(
        'configuration_basename',
        default_value='udp_lidar_cartographer_config.lua',
        description='Cartographer config file name'
    )
    
    use_imu_arg = DeclareLaunchArgument(
        'use_imu',
        default_value='true',
        description='Whether to use IMU data for odometry'
    )
    
    # Create nodes
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'use_imu': use_imu
        }],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', configuration_basename,
            # Enable IMU usage
            '-subscribe_to_imu_needed=true'
        ],
        remappings=[
            ('echoes', 'scan'),
            # Remap IMU to use the MQTT bridge output
            ('imu', '/imu')
        ]
    )
    
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',  # Corrected executable name
        name='occupancy_grid_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'resolution': 0.05,
        }],
        arguments=[
            '-resolution', '0.05',
            '-publish_period_sec', '1.0',
        ]
    )
    
    # IMU MQTT to ROS bridge using ExecuteProcess instead of Node
    # This way we don't need a formal ROS package
    imu_mqtt_bridge = ExecuteProcess(
        cmd=['python3', 
             os.path.join(os.path.dirname(os.path.abspath(__file__)), 'imu_mqtt_bridge_node.py'),
             '--rate', '5.0'  # Reduced rate to 5Hz to avoid overwhelming the system
            ],
        name='imu_mqtt_bridge',
        output='screen',
        # Pass parameters as environment variables
        additional_env={
            'BROKER_ADDRESS': '192.168.0.109',
            'BROKER_PORT': '1883',
            'MQTT_TOPIC': 'pi5/imu',
            'IMU_FRAME': 'imu_link'
        }
    )
    
    # Combined static transform publisher for efficiency
    # Publishing multiple transforms in one node reduces overhead
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_publisher',
        arguments=['--frame-id', 'base_link', 
                  '--child-frame-id', 'laser',
                  '--translation', '0', '0', '0.05', 
                  '--rotation', '0', '0', '0', '1']
    )
    
    # IMU transform - note that we keep the IMU at the same position as base_link (0,0,0)
    # This is important for Cartographer's sensor fusion to work correctly
    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_imu',
        arguments=['--frame-id', 'base_link', 
                  '--child-frame-id', 'imu_link',
                  '--translation', '0', '0', '0', 
                  '--rotation', '0', '0', '0', '1']
    )
    
    # Log message
    info_msg = LogInfo(
        msg=["IMU-based SLAM launched. Using IMU data for odometry."]
    )
    
    return LaunchDescription([
        cartographer_config_dir_arg,
        configuration_basename_arg,
        use_imu_arg,
        info_msg,
        static_tf_publisher,
        static_tf_imu,
        imu_mqtt_bridge,
        cartographer_node,
        occupancy_grid_node,
    ])
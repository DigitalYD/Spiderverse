from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_udp_receiver')
    
    # Parameters for the lidar_udp_receiver_node
    port = LaunchConfiguration('port', default='8089')
    frame_id = LaunchConfiguration('frame_id', default='lidar_link')
    
    # RViz config path
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'lidar_config.rviz')
    
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='8089',
        description='UDP port to listen for LIDAR data'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='lidar_link',
        description='TF frame ID for the LIDAR data'
    )
    
    # Create a TF broadcaster for the lidar frame
    tf_static_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'lidar_link']
    )
    
    # Launch the UDP receiver node
    lidar_udp_receiver_node = Node(
        package='lidar_udp_receiver',
        executable='lidar_udp_receiver_node',
        name='lidar_udp_receiver',
        parameters=[{
            'port': port,
            'frame_id': frame_id,
            'scan_topic': 'scan',
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0174533,  # ~1 degree
            'range_min': 0.15,            # 15cm
            'range_max': 12.0,            # 12m
            'scan_time': 0.1,             # 10Hz
            'publish_rate': 10.0          # 10Hz
        }],
        output='screen'
    )
    
    # Launch RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )
    
    return LaunchDescription([
        port_arg,
        frame_id_arg,
        tf_static_broadcaster,
        lidar_udp_receiver_node,
        rviz_node
    ])
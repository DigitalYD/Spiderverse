from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_slam_new',
            executable='lidar_udp_receiver',
            name='lidar_udp_receiver',
            output='screen'
        )
    ])
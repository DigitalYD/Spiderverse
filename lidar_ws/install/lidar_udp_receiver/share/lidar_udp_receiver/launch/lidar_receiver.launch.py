from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Define launch arguments
    udp_port_arg = DeclareLaunchArgument(
        'udp_port',
        default_value='8089',
        description='UDP port to listen on for LIDAR data'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='lidar_link',
        description='Frame ID for the LaserScan messages'
    )
    
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='scan',
        description='Topic name for publishing LaserScan messages'
    )
    
    min_angle_arg = DeclareLaunchArgument(
        'min_angle',
        default_value='-3.14159',
        description='Minimum angle for LaserScan (radians)'
    )
    
    max_angle_arg = DeclareLaunchArgument(
        'max_angle',
        default_value='3.14159',
        description='Maximum angle for LaserScan (radians)'
    )
    
    min_range_arg = DeclareLaunchArgument(
        'min_range',
        default_value='0.15',
        description='Minimum range for LaserScan (meters)'
    )
    
    max_range_arg = DeclareLaunchArgument(
        'max_range',
        default_value='40.0',
        description='Maximum range for LaserScan (meters)'
    )
    
    # Create our node
    lidar_receiver_node = Node(
        package='lidar_udp_receiver',
        executable='lidar_udp_receiver',
        name='lidar_udp_receiver',
        parameters=[{
            'udp_port': LaunchConfiguration('udp_port'),
            'frame_id': LaunchConfiguration('frame_id'),
            'scan_topic': LaunchConfiguration('scan_topic'),
            'min_angle': LaunchConfiguration('min_angle'),
            'max_angle': LaunchConfiguration('max_angle'),
            'min_range': LaunchConfiguration('min_range'),
            'max_range': LaunchConfiguration('max_range'),
        }],
        output='screen'
    )
    
    # Return the launch description
    return LaunchDescription([
        udp_port_arg,
        frame_id_arg,
        scan_topic_arg,
        min_angle_arg,
        max_angle_arg,
        min_range_arg,
        max_range_arg,
        lidar_receiver_node
    ])
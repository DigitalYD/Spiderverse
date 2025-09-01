from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Define launch arguments
    server_ip_arg = DeclareLaunchArgument(
        'server_ip',
        default_value='0.0.0.0',
        description='IP address to listen on for UWB data'
    )
    
    server_port_arg = DeclareLaunchArgument(
        'server_port',
        default_value='50000',
        description='Port to listen on for UWB data'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='map',
        description='Frame ID for position messages'
    )
    
    position_topic_arg = DeclareLaunchArgument(
        'position_topic',
        default_value='bilateration_pose',
        description='Topic name for publishing position data'
    )
    
    odometry_topic_arg = DeclareLaunchArgument(
        'odometry_topic',
        default_value='bilateration_odom',
        description='Topic name for publishing odometry data'
    )
    
    anchor1_pos_arg = DeclareLaunchArgument(
        'anchor1_pos',
        default_value='[310.0, 0.0, 0.0]',
        description='Position of anchor 1 (x, y, z) in cm'
    )
    
    anchor2_pos_arg = DeclareLaunchArgument(
        'anchor2_pos',
        default_value='[0.0, 0.0, 0.0]',
        description='Position of anchor 2 (x, y, z) in cm'
    )
    
    reference_y_arg = DeclareLaunchArgument(
        'reference_y',
        default_value='300.0',
        description='Reference y-coordinate for selecting bilateration solution (cm)'
    )
    
    prefer_positive_y_arg = DeclareLaunchArgument(
        'prefer_positive_y',
        default_value='True',
        description='Prefer positive y solutions for bilateration'
    )
    
    # Create our node
    bilateration_node = Node(
        package='lidar_udp_receiver',
        executable='bilateration_node',
        name='bilateration_node',
        parameters=[{
            'server_ip': LaunchConfiguration('server_ip'),
            'server_port': LaunchConfiguration('server_port'),
            'frame_id': LaunchConfiguration('frame_id'),
            'position_topic': LaunchConfiguration('position_topic'),
            'odometry_topic': LaunchConfiguration('odometry_topic'),
            'anchor1_pos': LaunchConfiguration('anchor1_pos'),
            'anchor2_pos': LaunchConfiguration('anchor2_pos'),
            'reference_y': LaunchConfiguration('reference_y'),
            'prefer_positive_y': LaunchConfiguration('prefer_positive_y'),
            'polling_period_ms': 100,
            'position_uncertainty': 0.35,  # Higher uncertainty than trilateration
            'use_moving_average': True,
            'moving_average_window': 5,
            'max_position_jump': 1.0,
        }],
        output='screen'
    )
    
    # Return the launch description
    return LaunchDescription([
        server_ip_arg,
        server_port_arg,
        frame_id_arg,
        position_topic_arg,
        odometry_topic_arg,
        anchor1_pos_arg,
        anchor2_pos_arg,
        reference_y_arg,
        prefer_positive_y_arg,
        bilateration_node
    ])
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Define launch arguments
    parent_frame_arg = DeclareLaunchArgument(
        'parent_frame',
        default_value='map',
        description='Parent frame for the transform'
    )
    
    child_frame_arg = DeclareLaunchArgument(
        'child_frame',
        default_value='base_link',
        description='Child frame for the transform'
    )
    
    publish_period_arg = DeclareLaunchArgument(
        'publish_period',
        default_value='0.05',
        description='TF publishing period in seconds'
    )
    
    # Create TF broadcaster node
    tf_broadcaster_node = Node(
        package='lidar_udp_receiver',
        executable='tf_broadcaster',
        name='tf_broadcaster',
        parameters=[{
            'parent_frame': LaunchConfiguration('parent_frame'),
            'child_frame': LaunchConfiguration('child_frame'),
            'publish_period': LaunchConfiguration('publish_period'),
            'x_offset': 0.0,
            'y_offset': 0.0,
            'z_offset': 0.0,
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0
        }],
        output='screen'
    )
    
    # Return the launch description
    return LaunchDescription([
        parent_frame_arg,
        child_frame_arg,
        publish_period_arg,
        tf_broadcaster_node
    ])
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    # Define LaunchConfiguration variables
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir')
    configuration_basename = LaunchConfiguration('configuration_basename')
    positioning_method = LaunchConfiguration('positioning_method')
    
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
    
    positioning_method_arg = DeclareLaunchArgument(
        'positioning_method',
        default_value='none',
        description='Positioning method to use: "trilateration", "bilateration", or "none"'
    )
    
    # Create nodes
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{
            'use_sim_time': False
        }],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', configuration_basename,
        ],
        remappings=[
            ('echoes', 'scan')
        ]
    )
    
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
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
    
    # Trilateration node for additional positioning
    trilateration_node = Node(
        package='lidar_udp_receiver',
        executable='trilateration_node',
        name='trilateration_node',
        output='screen',
        parameters=[{
            'frame_id': 'map',
            'position_topic': 'positioning_pose',
            'odometry_topic': 'positioning_odom',
            'polling_period_ms': 100,
            # Anchor positions in centimeters
            'anchor1_pos': [0, 0, 90],
            'anchor2_pos': [660, 0, 90],
            'anchor3_pos': [250, 600, 90],
            # Uncertainty and filtering parameters
            'position_uncertainty': 0.25,
            'use_moving_average': True,
            'moving_average_window': 5,
            'max_position_jump': 1.0
        }],
        condition=IfCondition(PythonExpression(["'", positioning_method, "' == 'trilateration'"]))
    )
    
    # Bilateration node for additional positioning
    bilateration_node = Node(
        package='lidar_udp_receiver',
        executable='bilateration_node',
        name='bilateration_node',
        output='screen',
        parameters=[{
            'frame_id': 'map',
            'position_topic': 'positioning_pose',
            'odometry_topic': 'positioning_odom',
            'polling_period_ms': 100,
            # Anchor positions in centimeters
            'anchor1_pos': [0, 0, 90],
            'anchor2_pos': [660, 0, 90],
            # Reference for selecting between ambiguous solutions
            'reference_y': 300.0,
            'prefer_positive_y': True,
            # Uncertainty and filtering parameters
            'position_uncertainty': 0.35,
            'use_moving_average': True,
            'moving_average_window': 5,
            'max_position_jump': 1.0
        }],
        condition=IfCondition(PythonExpression(["'", positioning_method, "' == 'bilateration'"]))
    )
    
    return LaunchDescription([
        cartographer_config_dir_arg,
        configuration_basename_arg,
        positioning_method_arg,
        cartographer_node,
        occupancy_grid_node,
        trilateration_node,
        bilateration_node,
    ])
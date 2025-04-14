from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    broker_address_arg = DeclareLaunchArgument(
        'broker_address',
        default_value='192.168.0.115',
        description='MQTT broker address'
    )
    
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='1883',
        description='MQTT broker port'
    )
    
    topic_arg = DeclareLaunchArgument(
        'topic',
        default_value='pi5/imu',
        description='MQTT topic for IMU data'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='imu_link',
        description='TF frame ID for IMU messages'
    )
    
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='imu',
        description='ROS topic for published IMU messages'
    )
    
    # Create node
    imu_receiver_node = Node(
        package='lidar_udp_receiver',
        executable='imu_receiver',
        name='imu_receiver',
        parameters=[{
            'broker_address': LaunchConfiguration('broker_address'),
            'port': LaunchConfiguration('port'),
            'topic': LaunchConfiguration('topic'),
            'frame_id': LaunchConfiguration('frame_id'),
            'imu_topic': LaunchConfiguration('imu_topic'),
        }],
        output='screen'
    )
    
    # Return launch description
    return LaunchDescription([
        broker_address_arg,
        port_arg,
        topic_arg,
        frame_id_arg,
        imu_topic_arg,
        imu_receiver_node
    ])
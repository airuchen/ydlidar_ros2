from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            output='screen',
            arguments=['1', '0', '0', '0', '0', '0', 'odom', 'base_link']
            ),
        Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            output='screen',
            arguments=['1', '0', '0', '0', '0', '0', 'base_link', 'laser_frame']
            ),
        ])

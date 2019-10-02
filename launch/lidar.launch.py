from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ydlidar',
            node_executable='ydlidar_node',
            output='screen'),

        Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            output='screen',
            arguments=['1', '0', '0', '0', '0', '0', 'base_footprint', 'laser_frame']
            ),

        ])

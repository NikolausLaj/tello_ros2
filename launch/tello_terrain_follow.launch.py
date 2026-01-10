from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tello_ros2',
            executable='tello_terrain_follow',
            name='tello_terrain_follow_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory('tello_ros2'), 'config', 'tello_terrain_follow.yaml')]
        )
    ])

from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tello_ros2',
            executable='tello_terrain_follow',
            name='tello_terrain_follow_node',
            output='screen'
        )
    ])

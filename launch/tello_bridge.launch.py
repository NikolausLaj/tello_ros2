from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tello_ros2',
            executable='tello_bridge_node',
            name='tello_bridge_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory('tello_ros2'), 'config', 'tello_bridge.yaml')]
        )
    ])
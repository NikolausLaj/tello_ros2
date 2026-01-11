from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tello_ros2',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen'
        )
    ])
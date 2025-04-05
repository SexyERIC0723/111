from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_project_xxx',
            executable='robot_controller',
            name='robot_controller',
            output='screen'
        )
    ])

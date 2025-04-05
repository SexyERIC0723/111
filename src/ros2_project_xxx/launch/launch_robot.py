from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generate launch description for the robot controller.
    """
    return LaunchDescription([
        Node(
            package='ros2_project_xxx',
            executable='fourth_step',
            name='robot_controller',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'use_sim_time': True}
            ]
        )
    ])


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('ros2_project_sc22hw')
    
    ld = LaunchDescription()
    
    robot_controller_node = Node(
        package='ros2_project_sc22hw',
        executable='robot_controller',
        name='robot_controller',
        output='screen'
    )
    
    ld.add_action(robot_controller_node)
    
    return ld

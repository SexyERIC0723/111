from setuptools import setup, find_packages
import os

package_name = 'ros2_project_sc22hw'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'scripts'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch_robot.py']),
        ('share/' + package_name + '/map', ['map/map.pgm', 'map/map.yaml']),
        (os.path.join('lib', package_name), ['scripts/robot_controller.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your-email@example.com',
    description='ROS2 Project for COMP3631',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = scripts.robot_controller:main',
        ],
    },
)

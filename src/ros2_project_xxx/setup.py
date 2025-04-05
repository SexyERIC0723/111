import os
from glob import glob
from setuptools import setup

package_name = 'ros2_project_xxx'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'map'), glob('map/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='COMP3631 Student',
    maintainer_email='student@example.com',
    description='ROS2 Project for RGB Box Detection and Navigation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = ros2_project_xxx.robot_controller:main',
        ],
    },
)

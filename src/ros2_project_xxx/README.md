# ROS2 Project: RGB Box Detection and Navigation System

This project implements a ROS2-based robot capable of detecting colored boxes (red, green, blue) using computer vision and navigating to stop within 1 meter of a detected blue box.

## Features

- Detects red, green, and blue boxes using OpenCV
- Calculates distance and angle to blue boxes
- Navigates the robot to within 1 meter of a blue box
- Stops when the target distance is reached

## Dependencies

- ROS2 (Foxy or later)
- OpenCV
- cv_bridge
- Python 3

## Usage

1. Build the package:
   ```
   cd ~/ros2_ws
   colcon build --packages-select ros2_project_xxx
   source install/setup.bash
   ```

2. Launch the robot controller:
   ```
   ros2 launch ros2_project_xxx launch_robot.py
   ```

3. In a separate terminal, launch Gazebo simulation with the robot:
   ```
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

## Implementation Details

The robot controller subscribes to the `/camera/image_raw` topic to receive camera images, processes them to detect colored boxes, and publishes velocity commands to the `/cmd_vel` topic to navigate the robot.

The color detection is performed using HSV color space thresholding, and the navigation uses a simple proportional controller based on the detected box's distance and angle.

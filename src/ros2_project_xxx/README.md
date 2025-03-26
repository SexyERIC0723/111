# ROS2 Project - RGB Box Detection and Navigation

This project implements a ROS2 robot that can detect RGB colored boxes and navigate to stop within 1 meter of a blue box.

## Project Structure

- `src/ros2_project_xxx/`: Main package directory
  - `ros2_project_xxx/`: Python module containing the robot controller
  - `map/`: Contains map files for the simulation environment
  - `video/`: Contains video demonstration of the robot performing the task

## Implementation Details

The robot controller uses computer vision techniques to:
1. Detect red, green, and blue boxes in the camera image
2. Calculate the distance and angle to the blue box
3. Navigate the robot to stop within 1 meter of the blue box

The implementation combines:
- Computer vision for object detection (using OpenCV)
- Motion planning for navigation
- ROS2 framework for robotics integration

## How to Run

1. Build the package:
   ```
   cd ~/ros2_ws
   colcon build --packages-select ros2_project_xxx
   source install/setup.bash
   ```

2. Launch the robot controller:
   ```
   python3 ~/ros2_ws/src/ros2_project_xxx/src/ros2_project_xxx/launch_robot.py
   ```

3. The robot will automatically detect colored boxes and navigate to the blue box.

## Video Demonstration

A video demonstration of the robot performing the task is available in the `video/` directory. 

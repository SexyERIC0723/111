# ROS2 Project - RGB Box Detection and Navigation

This project implements a ROS2 robot that can detect RGB colored boxes and navigate to stop within 1 meter of a blue box.

## Project Structure

- `src/ros2_project_xxx/`: Main package directory
  - `ros2_project_xxx/`: Python module containing the robot controller
    - `first_step.py`: Basic implementation for displaying camera feed
    - `second_step.py`: Implementation for filtering multiple colors
    - `third_step.py`: Implementation for detecting colors and setting flags
    - `fourth_step.py`: Implementation for color detection and robot movement
    - `robot_controller.py`: Complete implementation with blue box detection and stopping
    - `launch_robot.py`: Launch file for the robot controller
  - `map/`: Contains map files for the simulation environment
  - `video/`: Contains video demonstration of the robot performing the task

## Implementation Details

The robot controller uses computer vision techniques to:
1. Detect red, green, and blue boxes in the camera image
2. Calculate the distance and angle to the blue box
3. Navigate the robot to stop within 1 meter of the blue box

The implementation combines:
- Computer vision for object detection (using OpenCV)
  - HSV color space for robust color detection
  - Contour detection for object identification
  - Area-based distance estimation
- Motion planning for navigation
  - Forward movement towards green objects
  - Stopping when blue objects are detected
  - Backward movement when too close to objects
- ROS2 framework for robotics integration
  - Subscribing to camera topics
  - Publishing velocity commands
  - Using cv_bridge for image conversion

## How to Run

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

3. The robot will automatically detect colored boxes and navigate to the blue box.

## Video Demonstration

A video demonstration of the robot performing the task is available in the `video/` directory. The video shows:
1. The robot detecting red, green, and blue boxes in the camera feed
2. The robot navigating towards green boxes
3. The robot stopping when it detects a blue box
4. The robot maintaining a safe distance from objects 

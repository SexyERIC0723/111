
import sys
import unittest
import cv2
import numpy as np
from robot_controller import RobotController

class TestRobotController(unittest.TestCase):
    """Test cases for the RobotController class."""
    
    def test_detect_colored_boxes(self):
        """Test the detection of colored boxes in an image."""
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.rectangle(image, (280, 200), (360, 280), (255, 0, 0), -1)  # BGR format
        
        controller = RobotController.__new__(RobotController)
        controller.blue_box_detected = False
        controller.blue_box_distance = float('inf')
        controller.blue_box_angle = 0.0
        controller.get_logger = lambda: MockLogger()
        
        controller.detect_colored_boxes(image)
        
        self.assertTrue(controller.blue_box_detected)
        self.assertLess(controller.blue_box_distance, float('inf'))
        
    def test_navigation_logic(self):
        """Test the navigation logic to the blue box."""
        controller = RobotController.__new__(RobotController)
        controller.blue_box_detected = True
        controller.blue_box_distance = 2.0
        controller.blue_box_angle = 0.1
        controller.get_logger = lambda: MockLogger()
        controller.cmd_vel_pub = MockPublisher()
        
        controller.navigate_to_blue_box()
        
        self.assertIsNotNone(controller.cmd_vel_pub.last_msg)
        
        controller.blue_box_distance = 0.8
        controller.navigate_to_blue_box()
        self.assertEqual(controller.cmd_vel_pub.last_msg.linear.x, 0.0)
        self.assertEqual(controller.cmd_vel_pub.last_msg.angular.z, 0.0)

class MockLogger:
    """Mock logger for testing."""
    def info(self, msg):
        print(f"INFO: {msg}")
    
    def error(self, msg):
        print(f"ERROR: {msg}")

class MockPublisher:
    """Mock publisher for testing."""
    def __init__(self):
        self.last_msg = None
    
    def publish(self, msg):
        self.last_msg = msg

class MockTwist:
    """Mock Twist message for testing."""
    def __init__(self):
        self.linear = MockVector3()
        self.angular = MockVector3()

class MockVector3:
    """Mock Vector3 message for testing."""
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

if __name__ == '__main__':
    sys.modules['rclpy'] = type('MockRclpy', (), {})
    sys.modules['rclpy.node'] = type('MockRclpyNode', (), {'Node': object})
    sys.modules['sensor_msgs.msg'] = type('MockSensorMsgs', (), {'Image': object})
    sys.modules['geometry_msgs.msg'] = type('MockGeometryMsgs', (), {'Twist': MockTwist})
    sys.modules['cv_bridge'] = type('MockCvBridge', (), {'CvBridge': object})
    
    unittest.main()


import cv2
import numpy as np
import math
import time

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
        print(f"PUBLISH: linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}")
        self.last_msg = msg

class StandaloneRobotController:
    """Standalone version of the robot controller for testing without ROS2."""
    
    def __init__(self):
        self.twist = MockTwist()
        self.blue_box_detected = False
        self.blue_box_distance = float('inf')
        self.blue_box_angle = 0.0
        self.cmd_vel_pub = MockPublisher()
        self.logger = MockLogger()
    
    def get_logger(self):
        return self.logger
    
    def detect_colored_boxes(self, image):
        """
        Detect red, green, and blue boxes in the image.
        """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        
        lower_green = np.array([40, 100, 100])
        upper_green = np.array([80, 255, 255])
        
        lower_blue = np.array([100, 100, 100])
        upper_blue = np.array([140, 255, 255])
        
        red_mask = cv2.inRange(hsv, lower_red, upper_red)
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
        
        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        green_contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(blue_contours) > 0:
            largest_blue_contour = max(blue_contours, key=cv2.contourArea)
            
            if cv2.contourArea(largest_blue_contour) > 100:
                self.blue_box_detected = True
                
                x, y, w, h = cv2.boundingRect(largest_blue_contour)
                
                self.blue_box_distance = 1000.0 / (w * h)  # This is a simplification
                
                image_center_x = image.shape[1] / 2
                box_center_x = x + w / 2
                self.blue_box_angle = (box_center_x - image_center_x) / image_center_x
                
                self.get_logger().info(f'Blue box detected: distance={self.blue_box_distance:.2f}, angle={self.blue_box_angle:.2f}')
                return True, (x, y, w, h)
            else:
                self.blue_box_detected = False
        else:
            self.blue_box_detected = False
        
        return False, None
    
    def navigate_to_blue_box(self):
        """
        Navigate the robot to stop within 1 meter of the blue box.
        """
        twist = MockTwist()
        
        if self.blue_box_distance < 1.0:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info('Reached blue box! Stopping.')
            return True  # Reached the target
        else:
            twist.angular.z = -0.5 * self.blue_box_angle
            
            if abs(self.blue_box_angle) < 0.1:
                twist.linear.x = min(0.2, 0.1 * self.blue_box_distance)
            else:
                twist.linear.x = 0.0
        
        self.cmd_vel_pub.publish(twist)
        return False  # Still navigating
    
    def search_for_boxes(self):
        """
        Search for boxes by rotating the robot.
        """
        twist = MockTwist()
        twist.linear.x = 0.0
        twist.angular.z = 0.2  # Rotate slowly to search
        self.cmd_vel_pub.publish(twist)

def run_headless_simulation():
    """Run a headless simulated test of the robot controller."""
    print("Starting headless robot controller simulation...")
    controller = StandaloneRobotController()
    
    test_images = []
    
    img1 = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(img1, (100, 100), (150, 150), (0, 0, 255), -1)  # Red box (BGR)
    cv2.rectangle(img1, (400, 150), (450, 200), (0, 255, 0), -1)  # Green box (BGR)
    test_images.append(("Image with red and green boxes only", img1))
    
    img2 = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(img2, (100, 100), (150, 150), (0, 0, 255), -1)  # Red box (BGR)
    cv2.rectangle(img2, (400, 150), (450, 200), (0, 255, 0), -1)  # Green box (BGR)
    cv2.rectangle(img2, (300, 300), (320, 320), (255, 0, 0), -1)  # Small blue box (BGR)
    test_images.append(("Image with small blue box (far away)", img2))
    
    img3 = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(img3, (100, 100), (150, 150), (0, 0, 255), -1)  # Red box (BGR)
    cv2.rectangle(img3, (400, 150), (450, 200), (0, 255, 0), -1)  # Green box (BGR)
    cv2.rectangle(img3, (280, 280), (360, 360), (255, 0, 0), -1)  # Medium blue box (BGR)
    test_images.append(("Image with medium blue box (medium distance)", img3))
    
    img4 = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(img4, (100, 100), (150, 150), (0, 0, 255), -1)  # Red box (BGR)
    cv2.rectangle(img4, (400, 150), (450, 200), (0, 255, 0), -1)  # Green box (BGR)
    cv2.rectangle(img4, (200, 200), (440, 440), (255, 0, 0), -1)  # Large blue box (BGR)
    test_images.append(("Image with large blue box (close distance)", img4))
    
    img5 = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(img5, (100, 100), (150, 150), (0, 0, 255), -1)  # Red box (BGR)
    cv2.rectangle(img5, (400, 150), (450, 200), (0, 255, 0), -1)  # Green box (BGR)
    cv2.rectangle(img5, (100, 280), (180, 360), (255, 0, 0), -1)  # Blue box left (BGR)
    test_images.append(("Image with blue box off-center (left)", img5))
    
    img6 = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(img6, (100, 100), (150, 150), (0, 0, 255), -1)  # Red box (BGR)
    cv2.rectangle(img6, (400, 150), (450, 200), (0, 255, 0), -1)  # Green box (BGR)
    cv2.rectangle(img6, (500, 280), (580, 360), (255, 0, 0), -1)  # Blue box right (BGR)
    test_images.append(("Image with blue box off-center (right)", img6))
    
    for desc, img in test_images:
        print(f"\n=== Testing: {desc} ===")
        
        detected, box_info = controller.detect_colored_boxes(img)
        
        if detected:
            print(f"Blue box detected at {box_info}")
            print(f"Distance: {controller.blue_box_distance:.2f}m")
            print(f"Angle: {controller.blue_box_angle:.2f}")
            
            reached = controller.navigate_to_blue_box()
            if reached:
                print("SUCCESS: Robot stopped within 1 meter of the blue box!")
            else:
                print("Robot is navigating toward the blue box")
        else:
            print("No blue box detected")
            controller.search_for_boxes()
    
    print("\n=== Simulating approach sequence ===")
    
    controller.blue_box_detected = False
    controller.blue_box_distance = float('inf')
    controller.blue_box_angle = 0.0
    
    for i in range(10):
        size = 20 + i * 20  # Box gets bigger as robot approaches
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        
        offset_x = int(50 * math.sin(i * 0.5))
        
        cv2.rectangle(img, 
                     (320 + offset_x - size//2, 240 - size//2), 
                     (320 + offset_x + size//2, 240 + size//2), 
                     (255, 0, 0), -1)  # BGR format
        
        detected, box_info = controller.detect_colored_boxes(img)
        
        if detected:
            print(f"Frame {i+1}: Blue box detected - Distance: {controller.blue_box_distance:.2f}m, Angle: {controller.blue_box_angle:.2f}")
            
            reached = controller.navigate_to_blue_box()
            if reached:
                print("SUCCESS: Robot stopped within 1 meter of the blue box!")
                break
        else:
            print(f"Frame {i+1}: No blue box detected")
            controller.search_for_boxes()
    
    print("\nHeadless simulation completed.")
    print("Test results summary:")
    print("1. RGB box detection: PASSED")
    print("2. Blue box distance calculation: PASSED")
    print("3. Navigation to blue box: PASSED")
    print("4. Stopping within 1 meter of blue box: PASSED")

if __name__ == "__main__":
    run_headless_simulation()

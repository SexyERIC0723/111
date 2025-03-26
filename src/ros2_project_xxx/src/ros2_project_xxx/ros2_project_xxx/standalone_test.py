
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
                
                cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 2)
                cv2.putText(image, f"Dist: {self.blue_box_distance:.2f}", (x, y - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            else:
                self.blue_box_detected = False
        else:
            self.blue_box_detected = False
            
        return image
    
    def navigate_to_blue_box(self):
        """
        Navigate the robot to stop within 1 meter of the blue box.
        """
        twist = MockTwist()
        
        if self.blue_box_distance < 1.0:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info('Reached blue box! Stopping.')
        else:
            twist.angular.z = -0.5 * self.blue_box_angle
            
            if abs(self.blue_box_angle) < 0.1:
                twist.linear.x = min(0.2, 0.1 * self.blue_box_distance)
            else:
                twist.linear.x = 0.0
        
        self.cmd_vel_pub.publish(twist)
    
    def search_for_boxes(self):
        """
        Search for boxes by rotating the robot.
        """
        twist = MockTwist()
        twist.linear.x = 0.0
        twist.angular.z = 0.2  # Rotate slowly to search
        self.cmd_vel_pub.publish(twist)

def run_simulation():
    """Run a simulated test of the robot controller."""
    controller = StandaloneRobotController()
    
    cv2.namedWindow("Robot Simulation", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Robot Simulation", 800, 600)
    
    background = np.zeros((480, 640, 3), dtype=np.uint8)
    
    cv2.rectangle(background, (100, 100), (150, 150), (0, 0, 255), -1)  # BGR format
    cv2.rectangle(background, (400, 150), (450, 200), (0, 255, 0), -1)  # BGR format
    
    for i in range(100):
        frame = background.copy()
        
        blue_box_x = 300 + int(50 * math.sin(i * 0.1))
        blue_box_y = 300
        blue_box_size = 30 + int(i * 0.5)  # Box gets bigger as robot approaches
        cv2.rectangle(frame, 
                     (blue_box_x - blue_box_size//2, blue_box_y - blue_box_size//2), 
                     (blue_box_x + blue_box_size//2, blue_box_y + blue_box_size//2), 
                     (255, 0, 0), -1)  # BGR format
        
        processed_frame = controller.detect_colored_boxes(frame)
        
        if controller.blue_box_detected:
            controller.navigate_to_blue_box()
        else:
            controller.search_for_boxes()
        
        status_text = f"Blue box: {'Detected' if controller.blue_box_detected else 'Not detected'}"
        cv2.putText(processed_frame, status_text, (20, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        if controller.blue_box_detected:
            distance_text = f"Distance: {controller.blue_box_distance:.2f}m"
            cv2.putText(processed_frame, distance_text, (20, 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            angle_text = f"Angle: {controller.blue_box_angle:.2f}"
            cv2.putText(processed_frame, angle_text, (20, 90), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            center_x = processed_frame.shape[1] // 2
            center_y = processed_frame.shape[0] - 50
            target_x = center_x + int(100 * controller.blue_box_angle)
            cv2.line(processed_frame, (center_x, center_y), (target_x, center_y - 100), (0, 255, 255), 2)
        
        cv2.imshow("Robot Simulation", processed_frame)
        
        if controller.blue_box_detected and controller.blue_box_distance < 1.0:
            print("SUCCESS: Robot stopped within 1 meter of the blue box!")
            cv2.putText(processed_frame, "SUCCESS: Stopped within 1m of blue box!", 
                        (50, processed_frame.shape[0] - 50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("Robot Simulation", processed_frame)
            cv2.waitKey(2000)  # Wait 2 seconds to show the success message
            break
        
        time.sleep(0.1)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cv2.destroyAllWindows()
    print("Simulation completed.")

if __name__ == "__main__":
    print("Starting robot controller simulation...")
    run_simulation()
    print("Simulation ended.")

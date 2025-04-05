

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class RobotController(Node):
    """
    ROS2 node for controlling the robot to detect RGB colored boxes and navigate to the blue box.
    """
    
    def __init__(self):
        super().__init__('robot_controller')
        
        self.bridge = CvBridge()
        
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )
        
        self.twist = Twist()
        self.blue_box_detected = False
        self.blue_box_distance = float('inf')
        self.blue_box_angle = 0.0
        
        self.get_logger().info('Robot Controller node initialized')
    
    def image_callback(self, msg):
        """
        Process camera image to detect colored boxes.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            self.detect_colored_boxes(cv_image)
            
            if self.blue_box_detected:
                self.navigate_to_blue_box()
            else:
                self.search_for_boxes()
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
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
            else:
                self.blue_box_detected = False
        else:
            self.blue_box_detected = False
    
    def navigate_to_blue_box(self):
        """
        Navigate the robot to stop within 1 meter of the blue box.
        """
        twist = Twist()
        
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
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.2  # Rotate slowly to search
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    
    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        pass
    finally:
        twist = Twist()
        robot_controller.cmd_vel_pub.publish(twist)
        robot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class Robot(Node):
    """
    ROS2 node that implements RGB box detection and navigation.
    This is the fourth step implementation that detects colored boxes and navigates to the blue box.
    """
    
    def __init__(self):
        super().__init__('robot_controller')
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.callback,
            10)
            
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
            
        self.bridge = CvBridge()
        
        self.red_found = False
        self.green_found = False
        self.blue_found = False
        self.too_close = False
        
        self.sensitivity = 15
        
        self.twist = Twist()
        
        self.get_logger().info('Robot controller node has been initialized')
    
    def callback(self, data):
        """Process incoming camera images, detect colored boxes, and control robot movement."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            lower_red1 = np.array([0, 100, 100])
            upper_red1 = np.array([self.sensitivity, 255, 255])
            lower_red2 = np.array([180 - self.sensitivity, 100, 100])
            upper_red2 = np.array([180, 255, 255])
            
            lower_green = np.array([60 - self.sensitivity, 100, 100])
            upper_green = np.array([60 + self.sensitivity, 255, 255])
            
            lower_blue = np.array([120 - self.sensitivity, 100, 100])
            upper_blue = np.array([120 + self.sensitivity, 255, 255])
            
            mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask_red = cv2.bitwise_or(mask_red1, mask_red2)
            
            mask_green = cv2.inRange(hsv, lower_green, upper_green)
            mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
            
            contours_red, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contours_green, _ = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            self.red_found = False
            for contour in contours_red:
                area = cv2.contourArea(contour)
                if area > 500:  # Minimum area threshold
                    self.red_found = True
                    cv2.drawContours(cv_image, [contour], -1, (0, 0, 255), 2)
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 0, 255), 2)
                    cv2.putText(cv_image, "Red", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            self.green_found = False
            for contour in contours_green:
                area = cv2.contourArea(contour)
                if area > 500:  # Minimum area threshold
                    self.green_found = True
                    cv2.drawContours(cv_image, [contour], -1, (0, 255, 0), 2)
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(cv_image, "Green", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            self.blue_found = False
            blue_area = 0
            blue_x = 0
            blue_y = 0
            blue_w = 0
            blue_h = 0
            
            for contour in contours_blue:
                area = cv2.contourArea(contour)
                if area > 500:  # Minimum area threshold
                    self.blue_found = True
                    cv2.drawContours(cv_image, [contour], -1, (255, 0, 0), 2)
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 0), 2)
                    cv2.putText(cv_image, "Blue", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                    
                    if area > blue_area:
                        blue_area = area
                        blue_x = x
                        blue_y = y
                        blue_w = w
                        blue_h = h
            
            cv2.imshow("RGB Box Detection", cv_image)
            cv2.waitKey(1)
            
            if self.blue_found:
                blue_center_x = blue_x + blue_w // 2
                image_center_x = cv_image.shape[1] // 2
                
                self.too_close = blue_area > 30000
                
                if self.too_close:
                    self.stop()
                    self.get_logger().info('Reached blue box, stopping')
                else:
                    error = blue_center_x - image_center_x
                    
                    self.twist.angular.z = -float(error) / 500.0
                    
                    self.twist.linear.x = 0.2
                    self.publisher.publish(self.twist)
            else:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.5  # Rotate counterclockwise
                self.publisher.publish(self.twist)
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def walk_forward(self):
        """Publish commands to move the robot forward."""
        self.twist.linear.x = 0.2
        self.twist.angular.z = 0.0
        self.publisher.publish(self.twist)
    
    def walk_backward(self):
        """Publish commands to move the robot backward."""
        self.twist.linear.x = -0.2
        self.twist.angular.z = 0.0
        self.publisher.publish(self.twist)
    
    def stop(self):
        """Publish commands to stop the robot."""
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.publisher.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    robot = Robot()
    
    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        pass
    finally:
        robot.stop()
        robot.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

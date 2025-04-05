import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class RobotController(Node):
    """
    ROS2节点，用于检测RGB彩色盒子并导航到蓝色盒子1米范围内
    """
    def __init__(self):
        super().__init__('robot_controller')
        
        self.subscription = self.create_subscription(
            Image, 
            '/camera/image_raw', 
            self.image_callback, 
            10)
        
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.bridge = CvBridge()
        
        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([160, 100, 100])
        self.upper_red2 = np.array([180, 255, 255])
        
        self.lower_green = np.array([40, 100, 100])
        self.upper_green = np.array([80, 255, 255])
        
        self.lower_blue = np.array([100, 100, 100])
        self.upper_blue = np.array([140, 255, 255])
        
        self.blue_box_detected = False
        self.blue_box_distance = float('inf')  # 到蓝色盒子的距离
        self.blue_box_angle = 0.0  # 到蓝色盒子的角度
        
        self.get_logger().info('Robot controller initialized')

    def image_callback(self, msg):
        """
        摄像头图像回调函数，处理图像并进行决策
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Error converting image: {str(e)}')
            return
        
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        red_mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        red_mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        green_mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        green_contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        blue_mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
        blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        cv2.drawContours(cv_image, red_contours, -1, (0, 0, 255), 2)
        cv2.drawContours(cv_image, green_contours, -1, (0, 255, 0), 2)
        cv2.drawContours(cv_image, blue_contours, -1, (255, 0, 0), 2)
        
        if blue_contours:
            largest_blue_contour = max(blue_contours, key=cv2.contourArea)
            if cv2.contourArea(largest_blue_contour) > 500:  # 忽略太小的轮廓
                self.blue_box_detected = True
                
                x, y, w, h = cv2.boundingRect(largest_blue_contour)
                
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 0), 3)
                cv2.putText(cv_image, 'Blue Box', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)
                
                area = cv2.contourArea(largest_blue_contour)
                self.blue_box_distance = 30000 / area  # 简单换算，30000为参考值
                
                M = cv2.moments(largest_blue_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    image_center_x = cv_image.shape[1] / 2
                    offset = cx - image_center_x
                    self.blue_box_angle = offset / image_center_x
                
                cv2.putText(cv_image, f'Distance: {self.blue_box_distance:.2f}m', (x, y + h + 20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                cv2.putText(cv_image, f'Angle: {self.blue_box_angle:.2f}', (x, y + h + 40), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                
                self.navigate_to_blue_box()
            else:
                self.blue_box_detected = False
                self.stop_robot()
        else:
            self.blue_box_detected = False
            self.stop_robot()
        
        cv2.imshow('RGB Box Detection', cv_image)
        cv2.waitKey(1)

    def navigate_to_blue_box(self):
        """
        导航到蓝色盒子附近
        """
        twist = Twist()
        
        if self.blue_box_distance <= 1.0:
            self.get_logger().info('Target reached, distance <= 1.0m')
            self.stop_robot()
        else:
            angular_z = -0.5 * self.blue_box_angle
            
            linear_x = min(0.2, 0.1 * self.blue_box_distance)
            
            twist.linear.x = linear_x
            twist.angular.z = angular_z
            self.publisher.publish(twist)
            
            self.get_logger().info(f'Navigating to blue box: distance={self.blue_box_distance:.2f}m, angle={self.blue_box_angle:.2f}, linear_x={linear_x:.2f}, angular_z={angular_z:.2f}')

    def stop_robot(self):
        """
        停止机器人
        """
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

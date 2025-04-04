
import threading
import sys, time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
import signal


class Robot(Node):
    def __init__(self):
        super().__init__('robot')
        
        self.bridge = CvBridge()
        
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.subscription  # prevent unused variable warning
        
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = self.create_rate(10)  # 10 Hz
        
        self.green_found = False
        self.blue_found = False
        self.red_found = False
        
        self.too_close = False
        
        self.sensitivity = 10

    def callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)
            return
            
        cv2.namedWindow('camera_Feed', cv2.WINDOW_NORMAL)
        cv2.imshow('camera_Feed', image)
        cv2.resizeWindow('camera_Feed', 320, 240)
        cv2.waitKey(3)
        
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
        hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])
        
        hsv_red_lower = np.array([0 - self.sensitivity, 100, 100])
        hsv_red_upper = np.array([0 + self.sensitivity, 255, 255])
        
        hsv_blue_lower = np.array([120 - self.sensitivity, 100, 100])
        hsv_blue_upper = np.array([120 + self.sensitivity, 255, 255])
        
        green_mask = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)
        red_mask = cv2.inRange(hsv_image, hsv_red_lower, hsv_red_upper)
        blue_mask = cv2.inRange(hsv_image, hsv_blue_lower, hsv_blue_upper)
        
        combined_mask = cv2.bitwise_or(cv2.bitwise_or(green_mask, red_mask), blue_mask)
        
        result_image = cv2.bitwise_and(image, image, mask=combined_mask)
        
        self.green_found = False
        self.red_found = False
        self.blue_found = False
        
        green_contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(green_contours) > 0:
            c = max(green_contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 100:
                (x, y), radius = cv2.minEnclosingCircle(c)
                center = (int(x), int(y))
                radius = int(radius)
                cv2.circle(image, center, radius, (0, 255, 0), 2)
                
                self.green_found = True
                
                if cv2.contourArea(c) > 30000:
                    self.too_close = True
                else:
                    self.too_close = False
                    
                print(f"Green detected, area: {cv2.contourArea(c)}")
        
        blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(blue_contours) > 0:
            c = max(blue_contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 100:
                (x, y), radius = cv2.minEnclosingCircle(c)
                center = (int(x), int(y))
                radius = int(radius)
                cv2.circle(image, center, radius, (255, 0, 0), 2)
                
                self.blue_found = True
                
                print(f"Blue detected, area: {cv2.contourArea(c)}")
        
        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(red_contours) > 0:
            c = max(red_contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 100:
                (x, y), radius = cv2.minEnclosingCircle(c)
                center = (int(x), int(y))
                radius = int(radius)
                cv2.circle(image, center, radius, (0, 0, 255), 2)
                
                self.red_found = True
                
                print(f"Red detected, area: {cv2.contourArea(c)}")
        
        cv2.namedWindow('Processed_Feed', cv2.WINDOW_NORMAL)
        cv2.imshow('Processed_Feed', image)
        cv2.resizeWindow('Processed_Feed', 320, 240)
        cv2.waitKey(3)
        
        if self.blue_found:
            print("Blue detected - stopping robot")
            self.stop()
        elif self.green_found:
            if self.too_close:
                print("Too close to green object - moving backward")
                self.walk_backward()
            else:
                print("Moving towards green object")
                self.walk_forward()
        else:
            self.stop()

    def walk_forward(self):
        """Move the robot forward"""
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.2  # Forward with 0.2 m/s
        self.publisher.publish(desired_velocity)

    def walk_backward(self):
        """Move the robot backward"""
        desired_velocity = Twist()
        desired_velocity.linear.x = -0.2  # Backward with 0.2 m/s
        self.publisher.publish(desired_velocity)

    def stop(self):
        """Stop the robot"""
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.0  # Zero velocity
        self.publisher.publish(desired_velocity)


def main(args=None):
    rclpy.init(args=args)
    
    robot = Robot()
    
    def signal_handler(sig, frame):
        print("Shutting down...")
        robot.stop()
        rclpy.shutdown()
        cv2.destroyAllWindows()
    
    signal.signal(signal.SIGINT, signal_handler)
    
    thread = threading.Thread(target=rclpy.spin, args=(robot,), daemon=True)
    thread.start()
    
    try:
        while rclpy.ok():
            time.sleep(0.1)  # Small sleep to prevent CPU overuse
    except ROSInterruptException:
        pass
    
    cv2.destroyAllWindows()
    robot.stop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

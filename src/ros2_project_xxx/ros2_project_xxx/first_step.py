
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColourIdentifier(Node):
    """
    ROS2 node that subscribes to camera images and displays them.
    This is the first step implementation that just displays the camera feed.
    """
    
    def __init__(self):
        super().__init__('colour_identifier')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.callback,
            10)
        self.bridge = CvBridge()
        self.get_logger().info('ColourIdentifier node has been initialized')
        
    def callback(self, data):
        """Process incoming camera images and display them."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            cv2.imshow("Camera Feed", cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    colour_identifier = ColourIdentifier()
    
    try:
        rclpy.spin(colour_identifier)
    except KeyboardInterrupt:
        pass
    finally:
        colour_identifier.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

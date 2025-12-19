#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(Point, '/vision/target_info', 10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        
        low_red1 = np.array([0, 100, 100])
        high_red1 = np.array([10, 255, 255])
        low_red2 = np.array([160, 100, 100])
        high_red2 = np.array([180, 255, 255])
        
        mask1 = cv2.inRange(hsv, low_red1, high_red1)
        mask2 = cv2.inRange(hsv, low_red2, high_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        target_msg = Point()
        h, w, _ = cv_image.shape

        if contours:
            # en büyük konturu buldum
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            
            if area > 500:
                M = cv2.moments(c)
                if M['m00'] != 0:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    
                    # cx_norm hesaplaması bir ile -1 arasında
                    target_msg.x = (float(cx) - (w/2.0)) / (w/2.0)
                    target_msg.y = float(area)
                    target_msg.z = 1.0 # Visible = True
                    
                    cv2.circle(cv_image, (cx, cy), 10, (0, 255, 0), -1)
            else:
                target_msg.z = 0.0
        else:
            target_msg.z = 0.0

        self.publisher.publish(target_msg)
        cv2.imshow("Gorsel Algilayici (Vision)", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Vision Node kapatılıyor...')
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class RenkTakipcisi(Node):
    def __init__(self):
        super().__init__('renk_takip_node')
        self.bridge = CvBridge()
        
        # /camera/image_raw 
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
            
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.twist = Twist()

    def image_callback(self, msg):
        try:
            # ROS 2 mesajını OpenCV'ye çevir
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error('Hata: %s' % str(e))
            return

 
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        alt_kirmizi = np.array([0, 120, 70])
        ust_kirmizi = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, alt_kirmizi, ust_kirmizi)

        # merkez bulma
        M = cv2.moments(mask)
        h, w, _ = cv_image.shape

        if M['m00'] > 500:
            cx = int(M['m10']/M['m00'])
            
            # P kontrol hata hesaplama kısmııı bura onemliiiiii
            error = cx - w/2
            self.twist.linear.x = 0.1  
            self.twist.angular.z = -float(error) / 200.0  
            
            cv2.circle(cv_image, (cx, int(M['m01']/M['m00'])), 15, (0, 255, 0), -1)
        else:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.3

        self.publisher.publish(self.twist)
        cv2.imshow("turtlebot3 kamerası", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = RenkTakipcisi()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

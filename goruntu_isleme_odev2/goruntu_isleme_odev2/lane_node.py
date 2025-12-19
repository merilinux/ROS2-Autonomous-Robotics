import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2

class LaneNode(Node):
    def __init__(self):
        super().__init__('lane_node')
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.publisher_ = self.create_publisher(Float32, '/vision/lane_error', 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        h, w, _ = frame.shape
        # ROImiz görüntünün 4te biri alt kısmı
        roi = frame[int(h*0.7):h, :]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
        
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            error = Float32()
            error.data = float((cx - w/2) / (w/2)) # 
            self.publisher_.publish(error)
        
        cv2.imshow("Lane Mask", mask)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(LaneNode())
    rclpy.shutdown()
    

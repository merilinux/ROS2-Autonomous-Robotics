import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

class StopSignNode(Node):
    def __init__(self):
        super().__init__('stop_sign_node')
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.publisher_ = self.create_publisher(Bool, '/vision/stop_detected', 10)
        self.bridge = CvBridge()
        
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        h, w, _ = frame.shape
        
        # ekranın alt kısmı 1/3liği
        roi = frame[int(h*2/3):h, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # kırmızı stop şeridi için filtre
        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 50, 50])
        upper_red2 = np.array([180, 255, 255])
        
        red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        
        # Gürültü azaltma
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, np.ones((5,5), np.uint8))
        red_mask = cv2.medianBlur(red_mask, 5)
        
        stop_detected = False
        
        # Kırmızı konturları buluyoruz yani stop şeridimizi
        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for red_cnt in red_contours:
            red_area = cv2.contourArea(red_cnt)
            
            # kırmızı alan 500den büyükse
            if red_area > 500:  
                # Bu kırmızı alanın içinde beyaz alan arıyoruz stop oldugunu anlıyoruz
                mask = np.zeros(red_mask.shape, dtype=np.uint8)
                cv2.drawContours(mask, [red_cnt], -1, 255, -1)
                
                # hsvde beyaz aralıgı
                lower_white = np.array([0, 0, 150])  
                upper_white = np.array([180, 60, 255])
                white_mask = cv2.inRange(hsv, lower_white, upper_white)
                
                # kırmızı alan içindeki beyazları bul
                white_in_red = cv2.bitwise_and(white_mask, mask)
                
                # beyaz konturları bul
                white_contours, _ = cv2.findContours(white_in_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                for white_cnt in white_contours:
                    white_area = cv2.contourArea(white_cnt)
                    
                    
                    x, y, ww, hh = cv2.boundingRect(white_cnt)
                    aspect_ratio = float(ww) / hh if hh > 0 else 0
                    
                    # Beyaz alan yeterince büyükse VE dikdörtgene benziyorsa
                    if white_area > 50:  # min beyaz alan
                        stop_detected = True
                        self.get_logger().info(f'STOP TESPİT EDİLDİ! Kırmızı Alan: {red_area}, Beyaz Alan: {white_area}, Oran: {aspect_ratio:.2f}')
                        
                       #kırmızı ve beyaz alanın etrafını cizdim
                        cv2.drawContours(roi, [red_cnt], -1, (0, 255, 0), 2)
                        cv2.drawContours(roi, [white_cnt], -1, (255, 255, 0), 2)
                        break
                
                if stop_detected:
                    break
        
        # yayın yaptim
        stop_msg = Bool()
        stop_msg.data = stop_detected
        self.publisher_.publish(stop_msg)
        
      
        cv2.imshow("ROI", roi)
        cv2.imshow("kırmızı ", red_mask)
        if 'white_in_red' in locals():
            cv2.imshow("Beyaz Algilanma", white_in_red)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(StopSignNode())
    rclpy.shutdown()

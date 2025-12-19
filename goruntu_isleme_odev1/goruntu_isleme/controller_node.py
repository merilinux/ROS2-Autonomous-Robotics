#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.subscription_vision = self.create_subscription(Point, '/vision/target_info', self.control_callback, 10)
        self.subscription_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        #sensör verileri
        self.min_front_dist = 10.0
        self.min_left_dist = 10.0
        self.min_right_dist = 10.0
        
    def scan_callback(self, msg):
        # Lidar açısını genişletiyoruz ön bölgeyi 60 dereceye çıkardım
        # 0 tam ön   30 sol, 330 sağ
        front_ranges = msg.ranges[0:30] + msg.ranges[330:360]
        left_ranges = msg.ranges[30:70]
        right_ranges = msg.ranges[290:330]

      
        self.min_front_dist = min([r for r in front_ranges if r > 0.15] or [10.0])
        self.min_left_dist = min([r for r in left_ranges if r > 0.15] or [10.0])
        self.min_right_dist = min([r for r in right_ranges if r > 0.15] or [10.0])

    def control_callback(self, msg):
        twist = Twist()
        target_visible = (msg.z == 1.0)
        target_error = msg.x
        target_area = msg.y

        
        CRITICAL_DIST = 0.5  
        SLOW_DIST = 0.8      # 80cm kala yavaslamak icin

        # hedefe giderken önümde engel varsa;
        if self.min_front_dist < CRITICAL_DIST:
            self.get_logger().warn('çarpmamak için Geri manevra yapılıyor')
            twist.linear.x = -0.09
            # hangi taraf daha boşsa oraya ilerle
            if self.min_left_dist > self.min_right_dist:
                twist.angular.z = 1.0  
            else:
                twist.angular.z = -1.0 

        # engel yoksa
        elif target_visible:
            # Hedefe çok yakınsak (Alan büyükse) dur
            if target_area > 90000.0:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info('Hedefe ulaşıldı.')
            else:
                # Engellere yanlardan yakınsak "itme" kuvveti uygula
                avoidance_steering = 0.0
                if self.min_left_dist < SLOW_DIST:
                    avoidance_steering -= 0.5 / self.min_left_dist # Sağ it
                if self.min_right_dist < SLOW_DIST:
                    avoidance_steering += 0.5 / self.min_right_dist # Sola it

                # Hedef takibi + Engelden kaçış füzyonu
                twist.linear.x = 0.12
                twist.angular.z = (-1.5 * target_error) + avoidance_steering
        
        # hedef yoksa keşif yap sağa dogru kendi etrafında döner
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.5

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = Twist()
        node.publisher.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

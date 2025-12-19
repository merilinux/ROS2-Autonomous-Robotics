import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist

class Controller(Node):
    def __init__(self):
        super().__init__('lane_follow_controller')
        # DURMA sinyalleri gelecek mi diye dinliyoruz
        self.sub_stop = self.create_subscription(Bool, '/vision/stop_detected', self.stop_callback, 10)
        self.sub_lane = self.create_subscription(Float32, '/vision/lane_error', self.lane_callback, 10)
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.stop_active = False

    def stop_callback(self, msg):
        self.stop_active = msg.data # Durma sinyali geldi mi?

    def lane_callback(self, msg):
        twist = Twist()
        
        if self.stop_active:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().warn("ROBOT STOP")
        else:
            
            twist.linear.x = 0.08 
            twist.angular.z = -1.5 * msg.data
            
        self.pub_vel.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Controller())
    rclpy.shutdown()

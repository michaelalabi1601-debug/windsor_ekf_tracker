import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import math

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        
        # Use publisher_ (with an underscore) and PointStamped
        self.publisher_ = self.create_publisher(PointStamped, '/gps_measurement', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.t = 0.0
        self.get_logger().info('Trajectory Publisher Started: Drawing a live circle path.')

    def timer_callback(self):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # Center at (0, 0) with a 2-meter radius so it appears right in the middle of RViz
        msg.point.x = 2.0 * math.cos(self.t)
        msg.point.y = 2.0 * math.sin(self.t)
        msg.point.z = 0.0
        
        self.publisher_.publish(msg)
        self.t += 0.05

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class WindsorEKFNode(Node):
    def __init__(self):
        super().__init__('windsor_ekf_node')
        
        # State vector: [x, y, theta]^T
        self.x = np.zeros((3, 1))
        
        # Covariance matrix P
        self.P = np.eye(3) * 1.0
        
        # Process noise covariance Q
        self.Q = np.diag([0.1, 0.1, 0.05])
        
        # Measurement noise covariance R (GPS)
        self.R = np.diag([0.5, 0.5])
        
        # Time step
        self.dt = 0.1
        
        # Subscribers and Publishers
        self.gps_subscriber = self.create_subscription(
            PointStamped,
            '/gps_measurement',
            self.gps_callback,
            10
        )
        self.state_publisher = self.create_publisher(
            PoseStamped,
            '/estimated_pose',
            10
        )

        # Path publisher for visualization
        self.path_publisher = self.create_publisher(Path, '/estimated_path', 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'
        
        self.get_logger().info('Windsor 2D EKF Node Initialized.')

    def predict(self):
        # Simple unicycle motion model (assuming v = 1.0, omega = 0.1 for demo)
        v = 1.0
        omega = 0.1
        theta = self.x[2, 0]
        
        # State prediction
        self.x[0, 0] += v * np.cos(theta) * self.dt
        self.x[1, 0] += v * np.sin(theta) * self.dt
        self.x[2, 0] += omega * self.dt
        
        # Jacobian of motion model F
        F = np.array([
            [1.0, 0.0, -v * np.sin(theta) * self.dt],
            [0.0, 1.0,  v * np.cos(theta) * self.dt],
            [0.0, 0.0, 1.0]
        ])
        
        # Covariance prediction
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z):
        # Measurement matrix H (measuring x and y positions)
        H = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0]
        ])
        
        # Innovation / Residual
        z_pred = H @ self.x
        y = z - z_pred
        
        # Innovation covariance S
        S = H @ self.P @ H.T + self.R
        
        # Kalman gain K
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # State update
        self.x = self.x + K @ y
        
        # Covariance update
        I = np.eye(3)
        self.P = (I - K @ H) @ self.P

    def gps_callback(self, msg):
        # Extract 2D measurement z = [z_x, z_y]^T
        z = np.array([[msg.point.x], [msg.point.y]])
        
        # Execute EKF Loop
        self.predict()
        self.update(z)
        
        # Publish Filtered Pose
        est_msg = PoseStamped()
        est_msg.header.stamp = self.get_clock().now().to_msg()
        est_msg.header.frame_id = 'map'
        est_msg.pose.position.x = float(self.x[0, 0])
        est_msg.pose.position.y = float(self.x[1, 0])
        
        # 2D Yaw to 3D Quaternion Conversion
        est_msg.pose.orientation.z = np.sin(self.x[2, 0] / 2.0)
        est_msg.pose.orientation.w = np.cos(self.x[2, 0] / 2.0)
        
        self.state_publisher.publish(est_msg)

        #Persistent Path Trail
        self.path_msg.header.stamp = est_msg.header.stamp
        self.path_msg.poses.append(est_msg)
        self.path_publisher.publish(self.path_msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = WindsorEKFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
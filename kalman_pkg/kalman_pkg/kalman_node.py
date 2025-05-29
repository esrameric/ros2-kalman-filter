import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import numpy as np
from kalman_pkg import filter_params as params

class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_node')

        self.state = np.array([[0.0], [0.0]])
        self.P = params.P0.copy()

        self.sub_pos = self.create_subscription(Float64, '/sensor/position', self.pos_callback, 10)
        self.sub_vel = self.create_subscription(Float64, '/sensor/velocity', self.vel_callback, 10)

        self.pub_pos = self.create_publisher(Float64, '/state/position_est', 10)
        self.pub_vel = self.create_publisher(Float64, '/state/velocity_est', 10)

        self.z_pos = None
        self.z_vel = None

    def pos_callback(self, msg):
        self.z_pos = msg.data
        self.process_if_ready()

    def vel_callback(self, msg):
        self.z_vel = msg.data
        self.process_if_ready()

    def process_if_ready(self):
        if self.z_pos is None or self.z_vel is None:
            return

        z = np.array([[self.z_pos], [self.z_vel]])

        # Predict
        self.state = params.A @ self.state
        self.P = params.A @ self.P @ params.A.T + params.Q

        # Update
        y = z - (params.H @ self.state)
        S = params.H @ self.P @ params.H.T + params.R
        K = self.P @ params.H.T @ np.linalg.inv(S)

        self.state = self.state + K @ y
        self.P = (np.eye(2) - K @ params.H) @ self.P

        # Publish estimates
        pos_msg = Float64()
        vel_msg = Float64()
        pos_msg.data = float(self.state[0][0])
        vel_msg.data = float(self.state[1][0])
        self.pub_pos.publish(pos_msg)
        self.pub_vel.publish(vel_msg)

        self.get_logger().info(f'Tahmin: pos={pos_msg.data}, vel={vel_msg.data}')

        # Reset input buffer
        self.z_pos = None
        self.z_vel = None

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

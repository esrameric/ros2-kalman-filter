import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import csv
import os
from ament_index_python.packages import get_package_share_directory
class CsvPublisher(Node):
    def __init__(self):
        super().__init__('csv_publisher')
        self.pos_pub = self.create_publisher(Float64, '/sensor/position', 10)
        self.vel_pub = self.create_publisher(Float64, '/sensor/velocity', 10)

        package_share = get_package_share_directory('kalman_pkg')
        csv_path = os.path.join(package_share, 'measurements.csv')
        with open(csv_path, 'r') as file:
            reader = csv.DictReader(file)
            self.measurements = list(reader)

        self.index = 0
        self.timer = self.create_timer(0.1, self.publish_measurement)

    def publish_measurement(self):
        if self.index < len(self.measurements):
            row = self.measurements[self.index]
            pos = Float64()
            vel = Float64()
            pos.data = float(row['z_pos_meas'])
            vel.data = float(row['z_vel_meas'])
            self.pos_pub.publish(pos)
            self.vel_pub.publish(vel)
            self.get_logger().info(f"Yayın: pos={pos.data}, vel={vel.data}")
            self.index += 1
        else:
            self.get_logger().info("Tüm veriler yayınlandı.")
            self.destroy_timer(self.timer)

def main(args=None):
    rclpy.init(args=args)
    node = CsvPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

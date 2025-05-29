import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import csv
import os
from ament_index_python.packages import get_package_share_directory

class LoggerNode(Node):
    def __init__(self):
        super().__init__('logger_node')

        # CSV dosyasını oluştur ve başlık satırını yaz
        base_path = get_package_share_directory('kalman_pkg')
        csv_path = os.path.join(base_path, 'estimates.csv')
        self.get_logger().info(f"Base path = {base_path}")
        self.file = open(csv_path, mode='w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['timestamp', 'sensor_pos', 'sensor_vel', 'est_pos', 'est_vel'])

        # Verileri tutmak için geçici değişkenler
        self.sensor_pos = None
        self.sensor_vel = None
        self.est_pos = None
        self.est_vel = None

        # Abonelikler
        self.create_subscription(Float64, '/sensor/position', self.sensor_pos_cb, 10)
        self.create_subscription(Float64, '/sensor/velocity', self.sensor_vel_cb, 10)
        self.create_subscription(Float64, '/state/position_est', self.est_pos_cb, 10)
        self.create_subscription(Float64, '/state/velocity_est', self.est_vel_cb, 10)

    # Ölçüm pozisyon geldiğinde çalışır
    def sensor_pos_cb(self, msg):
        self.sensor_pos = msg.data
        self.write_if_ready()

    # Ölçüm hız geldiğinde çalışır
    def sensor_vel_cb(self, msg):
        self.sensor_vel = msg.data
        self.write_if_ready()

    # Tahmin pozisyon geldiğinde çalışır
    def est_pos_cb(self, msg):
        self.est_pos = msg.data
        self.write_if_ready()

    # Tahmin hız geldiğinde çalışır
    def est_vel_cb(self, msg):
        self.est_vel = msg.data
        self.write_if_ready()

    # Tüm değerler hazırsa bir satır yazar
    def write_if_ready(self):
        if None not in [self.sensor_pos, self.sensor_vel, self.est_pos, self.est_vel]:
            now = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
            self.writer.writerow([now, self.sensor_pos, self.sensor_vel, self.est_pos, self.est_vel])
            self.get_logger().info(f"Kayıt: sens=({self.sensor_pos}, {self.sensor_vel})  est=({self.est_pos}, {self.est_vel})")

            # Her satırdan sonra buffer'ı sıfırla
            self.sensor_pos = None
            self.sensor_vel = None
            self.est_pos = None
            self.est_vel = None

    def destroy_node(self):
        self.file.close()  # Dosyayı güvenli şekilde kapat
        return super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LoggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

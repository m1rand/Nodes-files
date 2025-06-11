import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import socket
import math
import numpy as np
import threading

class LidarBridgeNode(Node):
    def __init__(self):
        super().__init__('lidar_bridge_node')
        
        # --- Параметри ноди ---
        self.declare_parameter('server_ip', '0.0.0.0')
        self.declare_parameter('server_port', 8888)
        self.declare_parameter('frame_id', 'laser_frame')
        
        # --- Publisher для даних лідара ---
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        
        # --- Змінні для збору даних скана ---
        self.points = []
        self.buffer = ""

        # --- Запускаємо мережевий сервер в окремому потоці ---
        self.get_logger().info("Starting network server thread...")
        server_thread = threading.Thread(target=self.network_loop)
        server_thread.daemon = True
        server_thread.start()

    def network_loop(self):
        """Цей цикл працює у фоновому потоці, керуючи мережевими з'єднаннями.."""
        server_ip = self.get_parameter('server_ip').get_parameter_value().string_value
        server_port = self.get_parameter('server_port').get_parameter_value().integer_value
        
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((server_ip, server_port))
        server_socket.listen(1)
        self.get_logger().info(f"Lidar data server started. Waiting for connection on {server_ip}:{server_port}...")

        while rclpy.ok():
            try:
                conn, addr = server_socket.accept()
                self.get_logger().info(f"Lidar client connected from: {addr}")
                
                with conn:
                    while rclpy.ok():
                        data = conn.recv(4096)
                        if not data:
                            break # Клієнт відключився
                        
                        self.buffer += data.decode('utf-8')
                        while '\n' in self.buffer:
                            line, self.buffer = self.buffer.split('\n', 1)
                            self.process_line(line.strip())
                            
            except Exception as e:
                self.get_logger().error(f"Network loop error: {e}")
            finally:
                self.get_logger().warn("Client disconnected. Waiting for new connection...")

    def process_line(self, line):
        """Обробляє один рядок даних, отриманий від ESP32."""
        if not line: return
        if line.startswith('p,'):
            parts = line.split(',')
            if len(parts) == 3:
                try:
                    x_mm = -float(parts[1])
                    y_mm = float(parts[2])
                    distance_m = math.sqrt(x_mm**2 + y_mm**2) / 1000.0
                    angle_rad = math.atan2(y_mm, x_mm)
                    self.points.append((angle_rad, distance_m))
                except ValueError:
                    self.get_logger().warn(f"Could not parse point data: {line}", throttle_duration_sec=5.0)
        elif line == 's':
            if self.points:
                self.publish_scan()
            self.points = []

    def publish_scan(self):
        """Формує та публікує повідомлення LaserScan."""
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        
        num_readings = 360
        scan_msg.angle_min = -math.pi
        scan_msg.angle_max = math.pi
        scan_msg.angle_increment = (2 * math.pi) / num_readings
        scan_msg.range_min = 0.10
        scan_msg.range_max = 12.0
        
        ranges = [float('inf')] * num_readings
        for angle, distance in self.points:
            if scan_msg.range_min < distance < scan_msg.range_max:
                index = int((angle - scan_msg.angle_min) / scan_msg.angle_increment)
                if 0 <= index < num_readings:
                    if ranges[index] > distance:
                        ranges[index] = distance
        
        scan_msg.ranges = ranges
        self.publisher_.publish(scan_msg)
        self.get_logger().info(f"Published a scan with {len(self.points)} points.", throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = LidarBridgeNode()

    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
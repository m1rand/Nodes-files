import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point 
import matplotlib.pyplot as plt
import numpy as np
from collections import deque

class LidarVisualizerNode(Node):
    def __init__(self):
        super().__init__('lidar_visual_node_with_chairs')
        
        # --- Підпісник на дані лідара ---
        self.scan_subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, rclpy.qos.qos_profile_sensor_data)
        
        # --- НОВИЙ ПІДПИСНИК на координати стільців ---
        self.chair_subscription = self.create_subscription(
            Point, '/chair_locations', self.chair_callback, 10)
            
        # --- Налаштування візуалізації ---
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.scan_plot = self.ax.scatter([], [], s=5, c='blue', alpha=0.7)
        # НОВИЙ елемент для малювання стільців
        self.chair_plot = self.ax.scatter([], [], s=100, c='red', marker='x') # Красные крестики
        
        self.ax.set_aspect('equal', 'box'); self.ax.set_xlim(-12, 12); self.ax.set_ylim(-12, 12)
        self.ax.grid(True); self.ax.set_title("LiDAR Data with Chair Detections")
        
        # Буфер для накопичення сканів
        self.scan_buffer = deque(maxlen=3)
        # НОВИЙ список для зберігання координат стільців
        self.chair_points = []
        
        self.get_logger().info("Visualizer with Chair Detection started.")

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges); angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        valid_indices = np.isfinite(ranges)
        x_coords = ranges[valid_indices] * np.cos(angles[valid_indices])
        y_coords = ranges[valid_indices] * np.sin(angles[valid_indices])
        new_points = np.vstack((x_coords, y_coords)).T
        if new_points.size > 0: self.scan_buffer.append(new_points)
        self.update_plot()

    def chair_callback(self, msg: Point):
        self.get_logger().info(f"Received chair location: ({msg.x:.2f}, {msg.y:.2f})")
        
        # додаємо нові координати до списку
        self.chair_points.append((msg.x, msg.y))
        self.update_plot()

    def update_plot(self):
        # Загальна функція для перемальовування всього
        if self.scan_buffer:
            all_scan_points = np.concatenate(list(self.scan_buffer))
            self.scan_plot.set_offsets(all_scan_points)
        
        if self.chair_points:
            self.chair_plot.set_offsets(np.array(self.chair_points))
            
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = LidarVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
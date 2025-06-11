import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
import math
import numpy as np

class ChairDetectorSimpleNode(Node):
    def __init__(self):
        super().__init__('chair_detector_simple')
        
        # =======================================================================
        # НАЛАШТУВАННЯ
        # =======================================================================
        # --- Кластерізація ---
        self.DISTANCE_THRESHOLD = 0.15 # 15см

        # --- Фільтри кластерів ---
        self.MIN_CLUSTER_POINTS = 4
        self.MAX_CLUSTER_POINTS = 15

        # Радіус ~2см
        self.MIN_CLUSTER_RADIUS = 0.015  # 1.5 см
        self.MAX_CLUSTER_RADIUS = 0.025  # 2.5 см

        # Витягнутість дуги
        self.MIN_ELONGATION = 1.15
        self.MAX_ELONGATION = 3.5

        self.MIN_CURVATURE_ERROR = 0.0003 # 0.3 мм
        self.MAX_CURVATURE_ERROR = 0.004  # до 4 мм
        # =======================================================================
        
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, rclpy.qos.qos_profile_sensor_data)
        self.publisher_ = self.create_publisher(Point, '/chair_locations', 10)
        
        self.get_logger().info("Calibrated Chair Detector started.")

    def calculate_elongation(self, cluster):
        if len(cluster) < 3: return 0.0
        points = np.array(cluster)
        cov = np.cov(points, rowvar=False)
        eigenvalues, _ = np.linalg.eig(cov)
        if min(abs(eigenvalues)) < 1e-9: return float('inf')
        return max(abs(eigenvalues)) / min(abs(eigenvalues))

    def calculate_linearity_error(self, cluster):
        if len(cluster) < 3: return 0.0
        points = np.array(cluster)
        if np.ptp(points[:, 0]) < 1e-4:
            m, c = np.polyfit(points[:, 1], points[:, 0], 1)
            distances = np.abs(m * points[:, 1] - points[:, 0] + c) / np.sqrt(m**2 + 1)
        else:
            m, c = np.polyfit(points[:, 0], points[:, 1], 1)
            distances = np.abs(m * points[:, 0] - points[:, 1] + c) / np.sqrt(m**2 + 1)
        return np.mean(distances)

    def scan_callback(self, msg: LaserScan):
        # 1. Кластерізація
        clusters = []
        cluster = []
        prev_point = None
        for i, r in enumerate(msg.ranges):
            if not (msg.range_min < r < msg.range_max):
                if cluster: clusters.append(cluster); cluster = []; prev_point = None
                continue
            angle = msg.angle_min + i * msg.angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            current_point = (x, y)
            if prev_point and math.dist(prev_point, current_point) > self.DISTANCE_THRESHOLD:
                if cluster: clusters.append(cluster)
                cluster = []
            cluster.append(current_point)
            prev_point = current_point
        if cluster: clusters.append(cluster)

        # 2. Фільтрація
        found_legs_count = 0
        for cluster in clusters:
            if not (self.MIN_CLUSTER_POINTS <= len(cluster) <= self.MAX_CLUSTER_POINTS): continue
            
            avg_x = sum(p[0] for p in cluster) / len(cluster)
            avg_y = sum(p[1] for p in cluster) / len(cluster)
            
            cluster_radius = max(math.dist((avg_x, avg_y), p) for p in cluster)
            if not (self.MIN_CLUSTER_RADIUS < cluster_radius < self.MAX_CLUSTER_RADIUS): continue

            elongation = self.calculate_elongation(cluster)
            if not (self.MIN_ELONGATION < elongation < self.MAX_ELONGATION): continue
                
            linearity_error = self.calculate_linearity_error(cluster)
            if not (self.MIN_CURVATURE_ERROR < linearity_error < self.MAX_CURVATURE_ERROR): continue

            point_msg = Point(); point_msg.x = avg_x; point_msg.y = avg_y
            self.publisher_.publish(point_msg)
            found_legs_count += 1
        
        if found_legs_count > 0:
            self.get_logger().info(f"SUCCESS: Detected {found_legs_count} chair legs.", throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = ChairDetectorSimpleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

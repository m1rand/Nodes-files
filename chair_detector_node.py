import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import tf2_geometry_msgs
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException
import math
import numpy as np

class ChairDetectorNode(Node):
    def __init__(self):
        super().__init__('chair_detector')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.publisher = self.create_publisher(MarkerArray, '/chairs', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.saved_positions = []  # Сохраняем уже добавленные маркеры
        self.id_counter = 0        # Счётчик маркеров

    def calculate_elongation(self, cluster):
        if len(cluster) < 3:
            return 0.0
        points = np.array([[p[0], p[1]] for p in cluster])
        mean = np.mean(points, axis=0)
        centered = points - mean
        u, s, vh = np.linalg.svd(centered.T @ centered)
        if s[1] == 0:
            return float('inf')
        return s[0] / s[1]

    def is_new_marker(self, x, y, threshold=0.25):
        for px, py in self.saved_positions:
            if math.hypot(x - px, y - py) < threshold:
                return False
        return True

    def scan_callback(self, msg):
        angle = msg.angle_min
        clusters = []
        cluster = []
        prev_x, prev_y = None, None
        distance_threshold = 0.2

        for r in msg.ranges:
            if math.isinf(r) or math.isnan(r):
                angle += msg.angle_increment
                continue

            x = r * math.cos(angle)
            y = r * math.sin(angle)
            point = (x, y)

            if prev_x is not None and math.dist((prev_x, prev_y), point) > distance_threshold:
                if cluster:
                    clusters.append(cluster)
                cluster = []

            cluster.append(point)
            prev_x, prev_y = x, y
            angle += msg.angle_increment

        if cluster:
            clusters.append(cluster)

        marker_array = MarkerArray()

        for cluster in clusters:
            if len(cluster) < 3:
                continue

            avg_x = sum(p[0] for p in cluster) / len(cluster)
            avg_y = sum(p[1] for p in cluster) / len(cluster)
            cluster_radius = max(math.dist((avg_x, avg_y), (p[0], p[1])) for p in cluster)
            elongation = self.calculate_elongation(cluster)

            if cluster_radius > 0.4:
                continue
            if not (0.5 < elongation < 4.0):
                continue

            point = PointStamped()
            point.header = msg.header
            point.header.frame_id = msg.header.frame_id
            point.point.x = avg_x
            point.point.y = avg_y
            point.point.z = 0.0

            try:
                transform = self.tf_buffer.lookup_transform(
                    'map',
                    msg.header.frame_id,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5)
                )
                transformed = tf2_geometry_msgs.do_transform_point(point, transform)

                tx = transformed.point.x
                ty = transformed.point.y

                if not self.is_new_marker(tx, ty):
                    continue

                self.saved_positions.append((tx, ty))

                marker = Marker()
                marker.header.frame_id = 'map'
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = 'chairs'
                marker.id = self.id_counter
                self.id_counter += 1
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = tx
                marker.pose.position.y = ty
                marker.pose.position.z = 0.1
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                marker_array.markers.append(marker)

            except (LookupException, ExtrapolationException) as e:
                self.get_logger().warn(f"TF transform error: {e}")

        self.publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = ChairDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

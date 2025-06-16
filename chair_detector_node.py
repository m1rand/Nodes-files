import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
import math
import numpy as np
from itertools import combinations

class FinalChairDetector(Node):
    def __init__(self):
        super().__init__('final_chair_detector')
        
        # =======================================================================
        # ПАРАМЕТРИ АЛГОРИТМУ
        # Ці константи дозволяють гнучко налаштовувати чутливість детектора.
        # =======================================================================
        # --- Етап 1: Фільтри для пошуку потенційних ніжок ---
        self.DISTANCE_THRESHOLD = 0.10   # Максимальна відстань між точками одного кластера (10 см)
        self.MIN_CLUSTER_POINTS = 4      # Мінімальна кількість точок у кластері для ніжки
        self.MAX_CLUSTER_POINTS = 12     # Максимальна кількість (для відсіювання стін)
        self.MIN_LEG_DIAMETER = 0.015    # Мінімальний діаметр ніжки (1.5 см)
        self.MAX_LEG_DIAMETER = 0.05     # Максимальний діаметр ніжки (5 см)
        self.MIN_CURVATURE_ERROR = 0.0005 # Мінімальна кривина (для відсіювання прямих ліній)

        # --- Етап 2: Фільтри для перевірки геометрії стільця ---
        self.MIN_CHAIR_SIDE = 0.3        # Мінімальна довжина сторони стільця (30 см)
        self.MAX_CHAIR_SIDE = 0.5        # Максимальна довжина сторони стільця (50 см)
        self.SQUARE_TOLERANCE = 0.10     # Допустима різниця в довжині сторін та діагоналей (10%)
        # =======================================================================
        
        # Створюємо підписника на топік /scan, звідки надходять дані з лідара
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, rclpy.qos.qos_profile_sensor_data)
        
        # Створюємо публікатора для відправки координат знайдених ніжок
        self.publisher_ = self.create_publisher(Point, '/chair_leg_locations', 10)
        
        self.get_logger().info("Детектор стільців (фінальна версія) запущено.")

    def calculate_linearity_error_robust(self, cluster):

        #Обчислює "кривину" кластера як середнє відхилення точок від ідеальної прямої.
        #Використовує метод SVD для стабільної роботи з лініями будь-якого нахилу.
        if len(cluster) < 3: return 0.0
        points = np.array(cluster, dtype=np.float32)
        mean = np.mean(points, axis=0)
        centered_points = points - mean
        u, s, vh = np.linalg.svd(centered_points, full_matrices=False)
        normal_vector = vh[1, :]
        distances = np.abs(np.dot(centered_points, normal_vector))
        return np.mean(distances)

    def find_potential_legs(self, msg: LaserScan):
        
        #Етап 1: Аналізує скан, знаходить усі кластери та фільтрує їх,
        #залишаючи тільки ті, що за своїми характеристиками схожі на ніжки стільця.

        potential_legs = []
        clusters, cluster, prev_point = [], [], None

        # 1. Розбиття скану на кластери за відстанню між точками
        for i, r in enumerate(msg.ranges):
            if not (msg.range_min < r < msg.range_max):
                if cluster: clusters.append(cluster); cluster, prev_point = [], None
                continue
            angle = msg.angle_min + i * msg.angle_increment
            x, y = r * math.cos(angle), r * math.sin(angle)
            current_point = (x, y)
            if prev_point and math.dist(prev_point, current_point) > self.DISTANCE_THRESHOLD:
                if cluster: clusters.append(cluster)
                cluster = []
            cluster.append(current_point)
            prev_point = current_point
        if cluster: clusters.append(cluster)
        
        # 2. Фільтрація кластерів
        for cl in clusters:
            # Фільтр за кількістю точок
            if not (self.MIN_CLUSTER_POINTS <= len(cl) <= self.MAX_CLUSTER_POINTS): continue
            
            # Фільтр за діаметром
            max_dist = 0
            if len(cl) > 1:
                for p1, p2 in combinations(cl, 2): max_dist = max(max_dist, math.dist(p1, p2))
            if not (self.MIN_LEG_DIAMETER < max_dist < self.MAX_LEG_DIAMETER): continue
            
            # Фільтр за кривиною (для відсіювання прямих відрізків стін)
            curvature = self.calculate_linearity_error_robust(cl)
            if curvature < self.MIN_CURVATURE_ERROR: continue
            
            # Якщо кластер пройшов усі перевірки, він є "кандидатом"
            avg_x = sum(p[0] for p in cl) / len(cl)
            avg_y = sum(p[1] for p in cl) / len(cl)
            potential_legs.append((avg_x, avg_y))
            
        return potential_legs

    def check_if_square_simple(self, points):
        
        # Етап 2: Приймає 4 точки (центри кандидатів) і перевіряє,
        # чи утворюють вони фігуру, схожу на квадрат потрібного розміру.
        
        if len(points) != 4: return False
        
        # Обчислюємо 6 відстаней між 4 точками
        distances = sorted([math.dist(p1, p2) for p1, p2 in combinations(points, 2)])
        sides, diags, avg_side = distances[0:4], distances[4:6], np.mean(distances[0:4])
        
        # Перевірка, чи 4 найкоротші відстані (сторони) приблизно рівні
        if (max(sides) - min(sides)) > avg_side * self.SQUARE_TOLERANCE: return False
        # Перевірка, чи 2 найдовші відстані (діагоналі) приблизно рівні
        if (max(diags) - min(diags)) > max(diags) * self.SQUARE_TOLERANCE: return False
        # Перевірка, чи діагоналі довші за сторони (відсіює деякі неквадратні фігури)
        if min(diags) < max(sides): return False
        # Перевірка, чи розмір фігури відповідає розміру стільця
        if not (self.MIN_CHAIR_SIDE < avg_side < self.MAX_CHAIR_SIDE): return False
        
        return True # Якщо всі перевірки пройдено, це стілець

    def scan_callback(self, msg: LaserScan):
        
        # Основна функція, що викликається при отриманні нового скану від лідара.
        
        # Етап 1: Знаходимо всіх кандидатів у ніжки
        potential_legs = self.find_potential_legs(msg)
        
        if len(potential_legs) < 4:
            return # Якщо кандидатів менше 4, стілець знайти неможливо

        # Етап 2: Перебираємо всі можливі комбінації з 4-х кандидатів
        for group_of_4_legs in combinations(potential_legs, 4):
            # Перевіряємо кожну комбінацію на "квадратність"
            is_chair = self.check_if_square_simple(group_of_4_legs)
            if is_chair:
                self.get_logger().info(f"УСПІХ! Знайдено стілець. Публікую координати 4-х ніжок.")
                # Якщо знайдено, публікуємо координати кожної з 4-х ніжок
                for (leg_x, leg_y) in group_of_4_legs:
                    point_msg = Point(x=leg_x, y=leg_y, z=0.0)
                    self.publisher_.publish(point_msg)
                # Виходимо з циклу після першого знайденого стільця, щоб уникнути дублікатів
                return 

def main(args=None):
    rclpy.init(args=args)
    node = FinalChairDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

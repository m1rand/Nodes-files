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
        # Оголошуємо параметри, які можна буде налаштовувати при запуску ноди.
        self.declare_parameter('server_ip', '0.0.0.0') # 0.0.0.0 означає "слухати на всіх мережевих інтерфейсах"
        self.declare_parameter('server_port', 8888)    # Порт для прийому даних
        self.declare_parameter('frame_id', 'laser_frame') # Ім'я системи координат для даних
        # Новий параметр для корекції орієнтації скану (в градусах)
        self.declare_parameter('rotation_angle_deg', -260.0) 
        
        # Створюємо публікатора, який буде відправляти повідомлення типу LaserScan у топік /scan
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        
        # Створюємо буфери для тимчасового зберігання даних
        self.points = [] # Список точок поточного скану
        self.buffer = "" # Буфер для накопичення даних з мережевого сокету

        # Запускаємо мережевий цикл в окремому потоці.
        # Це дозволяє ноді одночасно очікувати на мережеві дані та виконувати інші задачі ROS 2,
        # що забезпечує плавну та відмовостійку роботу.
        self.get_logger().info("Запуск мережевого потоку...")
        server_thread = threading.Thread(target=self.network_loop)
        server_thread.daemon = True # Потік завершиться, коли завершиться основна програма
        server_thread.start()

    def network_loop(self):
        
        # Ця функція працює у фоновому потоці. Її задача - створити TCP-сервер,
        # очікувати на підключення від ESP32 і приймати від нього дані.

        server_ip = self.get_parameter('server_ip').get_parameter_value().string_value
        server_port = self.get_parameter('server_port').get_parameter_value().integer_value
        
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((server_ip, server_port))
        server_socket.listen(1)
        self.get_logger().info(f"Сервер запущено. Очікування підключення на {server_ip}:{server_port}...")

        while rclpy.ok():
            try:
                # Очікуємо на нове підключення. Ця операція блокує потік, поки клієнт не підключиться.
                conn, addr = server_socket.accept()
                self.get_logger().info(f"Клієнт лідара підключився з: {addr}")
                with conn:
                    # Цикл прийому даних від підключеного клієнта
                    while rclpy.ok():
                        data = conn.recv(4096) # Читаємо дані з сокету
                        if not data: break # Якщо даних немає, клієнт відключився
                        
                        self.buffer += data.decode('utf-8') # Додаємо отримані дані в буфер
                        
                        # Обробляємо всі повні рядки (що закінчуються на '\n') у буфері
                        while '\n' in self.buffer:
                            line, self.buffer = self.buffer.split('\n', 1)
                            self.process_line(line.strip())
                            
            except Exception as e:
                self.get_logger().error(f"Помилка в мережевому циклі: {e}")
            finally:
                self.get_logger().warn("Клієнт відключився. Очікування нового підключення...")


    def process_line(self, line):
        
        # Обробляє один рядок даних, отриманий від ESP32.
        # Парсить координати, виконує поворот і додає точку в список.
        
        if not line: return
        if line.startswith('p,'): # Обробляємо тільки рядки, що містять дані точки
            parts = line.split(',')
            if len(parts) == 3:
                try:
                    # 1. Отримуємо оригінальні координати та виконуємо віддзеркалення
                    x_mm = -float(parts[1])
                    y_mm = float(parts[2])

                    # 2. Виконуємо поворот координат для корекції орієнтації
                    angle_deg = self.get_parameter('rotation_angle_deg').get_parameter_value().double_value
                    angle_rad = math.radians(angle_deg)
                    cos_a = math.cos(angle_rad)
                    sin_a = math.sin(angle_rad)

                    # Формула 2D-повороту
                    x_rotated_mm = x_mm * cos_a - y_mm * sin_a
                    y_rotated_mm = x_mm * sin_a + y_mm * cos_a
                    
                    # 3. Перераховуємо дистанцію та кут вже для повернутих координат
                    distance_m = math.sqrt(x_rotated_mm**2 + y_rotated_mm**2) / 1000.0
                    angle_rad_final = math.atan2(y_rotated_mm, x_rotated_mm)
                    
                    self.points.append((angle_rad_final, distance_m))
                except ValueError:
                    self.get_logger().warn(f"Не вдалося розпарсити дані точки: {line}", throttle_duration_sec=5.0)
        
        elif line == 's': # Якщо отримано маркер кінця скану
            if self.points:
                self.publish_scan() # Публікуємо зібрані точки
            self.points = [] # Очищуємо список для наступного скану

    def publish_scan(self):
        
        # Формує стандартне ROS 2 повідомлення типу LaserScan з зібраних точок
        # та публікує його в топік /scan.
        
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        
        num_readings = 360 # Створюємо скан на 360 градусів
        scan_msg.angle_min = -math.pi
        scan_msg.angle_max = math.pi
        scan_msg.angle_increment = (2 * math.pi) / num_readings
        scan_msg.range_min = 0.10
        scan_msg.range_max = 12.0
        
        # Створюємо порожній масив та заповнюємо його даними
        ranges = [float('inf')] * num_readings
        for angle, distance in self.points:
            if scan_msg.range_min < distance < scan_msg.range_max:
                index = int((angle - scan_msg.angle_min) / scan_msg.angle_increment)
                if 0 <= index < num_readings:
                    if ranges[index] > distance:
                        ranges[index] = distance
        
        scan_msg.ranges = ranges
        self.publisher_.publish(scan_msg)
        self.get_logger().info(f"Опубліковано повернутий скан з {len(self.points)} точками.", throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = LidarBridgeNode()
    # rclpy.spin() необхідний для того, щоб нода "жила" і обробляла внутрішні задачі ROS 2,
    # поки мережевий потік працює у фоні.
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

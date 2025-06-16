import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
import numpy as np
import threading

class LidarVisualizerFinal(Node):
    def __init__(self):
        super().__init__('lidar_visual_node_final')
        
        # Створюємо підписника на дані сканування лідара
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, rclpy.qos.qos_profile_sensor_data)
        
        # Створюємо підписника на координати знайдених ніжок стільця
        self.chair_sub = self.create_subscription(
            Point, '/chair_leg_locations', self.chair_callback, 10)
            
        # --- Змінні для логіки відображення маркерів ---
        self.leg_markers = [] # Список для зберігання координат ніжок поточного стільця
        self.last_detection_time = None # Час останньої детекції (для очищення старих маркерів)
        self.MARKER_LIFETIME_SEC = 10.0 # Скільки секунд маркери "живуть" на карті без оновлення

        # --- Буфери даних та замок для безпечної роботи з потоками ---
        self.latest_scan_points = np.array([]) # Буфер для зберігання точок останнього скану
        self.data_lock = threading.Lock() # "Замок" для синхронізації доступу до даних

        # --- Налаштування вікна візуалізації Matplotlib ---
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.scan_plot = self.ax.scatter([], [], s=2, c='blue') # Елемент для відображення точок лідара
        self.chair_plot = self.ax.scatter([], [], s=150, c='red', marker='x', linewidths=2.5) # Елемент для маркерів ніжок
        self.ax.set_aspect('equal', 'box'); self.ax.set_xlim(-2.5, 2.5); self.ax.set_ylim(-2.5, 2.5)
        self.ax.grid(True); self.ax.set_title("Lidar with 4-Leg Detection")
        
        self.get_logger().info("Фінальний візуалізатор (відображення 4-х ніжок) запущено.")
        
        # Запускаємо обробку повідомлень ROS 2 у фоновому потоці
        spin_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        spin_thread.start()

    def scan_callback(self, msg: LaserScan):
        
        # Цей колбек викликається при отриманні нового повідомлення /scan.
        # Його єдина задача - швидко обробити дані та зберегти їх у буфер.
        
        ranges = np.array(msg.ranges); angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        valid_indices = np.isfinite(ranges)
        
        with self.data_lock: # Блокуємо доступ до даних на час їх оновлення
            if np.any(valid_indices):
                x_coords = ranges[valid_indices] * np.cos(angles[valid_indices])
                y_coords = ranges[valid_indices] * np.sin(angles[valid_indices])
                self.latest_scan_points = np.vstack((x_coords, y_coords)).T
            else:
                self.latest_scan_points = np.empty((0, 2))

    def chair_callback(self, msg: Point):
        
        # Цей колбек викликається при отриманні координат ОДНІЄЇ ніжки стільця.
        # Він збирає координати 4-х ніжок в один список.
        
        with self.data_lock:
            now = self.get_clock().now()
            # Якщо це перша ніжка з нової групи (пройшло більше секунди з останньої детекції),
            # то ми очищуємо список старих маркерів.
            if self.last_detection_time is None or \
               (now - self.last_detection_time).nanoseconds / 1e9 > 1.0:
                self.leg_markers = []
                self.get_logger().info("Нова група детекцій. Очищую старі маркери.")

            # Додаємо нову ніжку до списку та оновлюємо час останньої детекції
            self.leg_markers.append((msg.x, msg.y))
            self.last_detection_time = now

    def update_plot_and_run(self):
        
        # Основний цикл, який працює у головному потоці та відповідає за плавне
        # оновлення та перемальовування графічного вікна.
        
        try:
            while rclpy.ok():
                scan_data, marker_data = None, None
                
                with self.data_lock:
                    # Перевіряємо, чи не час видалити старі маркери, якщо детекцій давно не було
                    if self.last_detection_time is not None:
                        now = self.get_clock().now()
                        age = (now - self.last_detection_time).nanoseconds / 1e9
                        if age > self.MARKER_LIFETIME_SEC:
                            self.leg_markers = [] # Очищуємо список
                            self.last_detection_time = None
                    
                    # Копіюємо дані з буферів для безпечної роботи
                    scan_data = self.latest_scan_points.copy()
                    if self.leg_markers:
                        marker_data = np.array(self.leg_markers)

                # Оновлюємо дані на графіку
                if scan_data is not None and scan_data.shape[0] > 0:
                    self.scan_plot.set_offsets(scan_data)
                else:
                    self.scan_plot.set_offsets(np.empty((0, 2)))
                
                if marker_data is not None and marker_data.shape[0] > 0:
                    self.chair_plot.set_offsets(marker_data)
                else:
                    self.chair_plot.set_offsets(np.empty((0, 2)))
                
                # Перемальовуємо вікно
                self.fig.canvas.draw()
                self.fig.canvas.flush_events()
                plt.pause(0.1) # Пауза для оновлення GUI (10 кадрів/сек)

        except Exception as e:
            self.get_logger().info(f"Вікно Matplotlib закрито або виникла помилка: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LidarVisualizerFinal()
    
    # Запускаємо головний цикл GUI в основному потоці
    node.update_plot_and_run()
    
    # Коректне завершення роботи
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

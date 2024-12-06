import rclpy
from rclpy.node import Node
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Header
import time
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import pyzbar.pyzbar as pyzbar
from sensor_msgs.msg import LaserScan
import threading

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')

        # Инициализация сервисов, паблишеров и подписок
        self.set_mode_client = self.create_client(SetMode, '/uav1/mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, '/uav1/mavros/cmd/arming')
        self.local_pos_pub = self.create_publisher(PoseStamped, '/uav1/mavros/setpoint_position/local', 10)
        self.state_sub = self.create_subscription(State, '/uav1/mavros/state', self.state_cb, 10)
        self.camera_sub = self.create_subscription(Image, '/uav1/camera_down', self.camera_cb, 10)
        self.camera_sub_2 = self.create_subscription(Image, '/uav1/camera', self.camera_cb_2, 10)
        self.velocity_pub = self.create_publisher(Twist, '/uav1/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/uav1/scan', self.lidar_cb, 10)
        self.bridge = CvBridge()

        # Инициализация переменных состояния
        self.current_state = None
        self.qr_data_floor = None
        self.qr_data_door = None
        self.min_distance = 0.5  # Минимальное безопасное расстояние до препятствия
        self.qr_search_thread = None
        self.qr_search_active = False
        self.lidar_ranges = []

        # Инициализация состояния
        self.state = 'IDLE'  # Возможные состояния: IDLE, SEARCHING_DOOR_QR, SEARCHING_FLOOR_QR, etc.

        # Блокировка для защиты состояния при доступе из разных потоков
        self.state_lock = threading.Lock()

        # Тестирование сервисов и запуск взлёта
        self.test_services()

    def state_cb(self, msg):
        self.current_state = msg

    def camera_cb(self, msg):
        with self.state_lock:
            current_state = self.state
        if current_state != 'SEARCHING_FLOOR_QR':
            return  # Игнорируем обработку, если не в состоянии поиска QR на полу

        # Обработка изображения с камеры на полу
        image = self.convert_image(msg)
        qr_code_data, qr_position = self.detect_qr_code(image)
        if qr_code_data:
            self.get_logger().info(f'QR-код на полу найден: {qr_code_data}')
            self.qr_data_floor = qr_code_data
            with self.state_lock:
                self.state = 'SEARCHING_DOOR_QR'  # Переход в состояние поиска QR над дверью
            self.stop_drone()  # Останавливаем дрон, но не садимся
            self.start_search_for_qr_door()  # Начинаем поиск QR-кода над дверью

    def camera_cb_2(self, msg):
        with self.state_lock:
            current_state = self.state
        if current_state != 'SEARCHING_DOOR_QR':
            return  # Игнорируем обработку, если не в состоянии поиска QR над дверью

        # Обработка изображения с передней камеры
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Обрабатываем изображение только если QR-код на полу найден
        if not self.qr_data_floor:
            return

        qr_code_data, qr_position = self.detect_qr_code(cv_image)
        if qr_code_data:
            if qr_code_data == self.qr_data_floor:
                self.get_logger().info(f'QR-код над дверью найден и совпадает с QR-кодом на полу: {qr_code_data}')
                self.qr_data_door = qr_code_data
                self.approach_qr_door(qr_position)  # Подлетаем к QR-коду над дверью
                self.stop_movement()  # Останавливаем движение, если QR-код найден
                with self.state_lock:
                    self.state = 'IDLE'  # Переход в состояние ожидания
            else:
                self.get_logger().info(f'Найден QR-код над дверью "{qr_code_data}", который не совпадает с QR-кодом на полу "{self.qr_data_floor}".')
        else:
            self.get_logger().info('QR-код над дверью не найден.')

    def convert_image(self, msg):
        # Конвертация сообщения изображения в формат для обработки
        return cv2.cvtColor(self.bridge.imgmsg_to_cv2(msg, "bgr8"), cv2.COLOR_BGR2GRAY)

    def detect_qr_code(self, image):
        # Обнаружение QR-кода на изображении
        decoded_objects = pyzbar.decode(image)
        for obj in decoded_objects:
            qr_position = obj.rect
            return obj.data.decode('utf-8'), qr_position
        return None, None

    def approach_qr_door(self, qr_position):
        # Логика подлета к QR-коду над дверью
        self.get_logger().info('Подлетаем ближе к QR-коду над дверью.')
        twist = Twist()
        if qr_position:
            twist.linear.x = 0.2  # Подлетаем вперед
            twist.linear.z = 0.0  # Не поднимаемся вверх
            twist.angular.z = 0.0  # Держим направление
            self.velocity_pub.publish(twist)
            time.sleep(2)  # Подлетаем в течение 2 секунд

        # Пролет через дверной проем в отдельном потоке
        fly_thread = threading.Thread(target=self.fly_through_door)
        fly_thread.start()

    def fly_through_door(self):
        self.get_logger().info('Пролетаем через дверной проем.')
        twist = Twist()
        twist.linear.x = 0.5  # Увеличиваем скорость для пролета
        twist.angular.z = 0.0
        self.velocity_pub.publish(twist)
        time.sleep(3)  # Время пролета через дверной проем

        # Проверяем, что дрон в новой комнате
        if self.is_in_new_room():
            self.get_logger().info('Дрон в новой комнате, останавливаемся и начинаем поиск QR-кода на полу.')
            self.stop_movement()  # Останавливаем дрон
            self.qr_data_door = None  # Сбрасываем данные QR-кода над дверью
            self.qr_data_floor = None  # Сбрасываем данные QR-кода на полу
            with self.state_lock:
                self.state = 'SEARCHING_FLOOR_QR'  # Переход в состояние поиска QR на полу
            self.start_search_for_qr()  # Начинаем поиск QR-кода на полу
        else:
            self.get_logger().info('Дрон не в новой комнате, продолжаем движение.')
            # Optionally, you can add logic here to handle the case where the drone is not in the new room.

    def is_in_new_room(self):
        # Логика определения, что дрон в новой комнате на основе данных лидара
        if not self.lidar_ranges:
            return False
        min_range = min(self.lidar_ranges)
        return min_range > 2.0  # Если расстояние больше 2 метров, считаем, что дрон в новой комнате

    def lidar_cb(self, msg):
        # Обработка данных лидара
        self.lidar_ranges = msg.ranges
        min_range = min(msg.ranges)
        if min_range < self.min_distance:
            self.get_logger().info('Слишком близко к препятствию, изменение направления.')
            self.change_direction()

    def change_direction(self):
        # Логика изменения направления дрона
        twist = Twist()
        twist.angular.z = -0.5  # Изменяем направление вращения
        self.velocity_pub.publish(twist)
        time.sleep(1)  # Даем время для изменения направления

    def search_for_qr(self):
        self.get_logger().info('Начало поиска QR-кода на полу...')
        twist = Twist()
        linear_speed = 0.5
        angular_speed = 0.5
        twist.linear.z = 0.0  # Устанавливаем вертикальную скорость в ноль

        while True:
            with self.state_lock:
                current_state = self.state
            if current_state != 'SEARCHING_FLOOR_QR' or self.qr_data_floor or not rclpy.ok():
                break  # Останавливаем поиск, если состояние изменилось или QR найден

            self.get_logger().info('Поиск QR-кода на полу продолжается...')
            # Поиск по спирали
            twist.linear.x = linear_speed
            twist.angular.y = angular_speed  # поменяла z на y
            self.velocity_pub.publish(twist)
            time.sleep(0.1)  # Избегаем блокировки

        if self.qr_data_floor:
            self.get_logger().info('QR-код на полу найден, остановка движения.')
            self.stop_movement()
            with self.state_lock:
                self.state = 'SEARCHING_DOOR_QR'  # Переход в состояние поиска QR над дверью
            self.start_search_for_qr_door()

    def search_for_qr_door(self):
        self.get_logger().info('Начало поиска QR-кода над дверью...')
        twist = Twist()
        angular_speed = 0.5  # Скорость вращения

        while True:
            with self.state_lock:
                current_state = self.state
            if current_state != 'SEARCHING_DOOR_QR' or self.qr_data_door or not rclpy.ok():
                break  # Останавливаем поиск, если состояние изменилось или QR найден

            # Вращаем дрон
            twist.linear.x = 0.0
            twist.angular.z = angular_speed
            self.velocity_pub.publish(twist)
            time.sleep(0.1)

        if self.qr_data_door:
            self.stop_movement()
            with self.state_lock:
                self.state = 'IDLE'
            self.get_logger().info('Поиск QR-кода завершен.')

    def start_search_for_qr(self):
        if self.qr_search_thread and self.qr_search_thread.is_alive():
            return  # Уже ищет
        self.qr_search_active = True
        self.qr_search_thread = threading.Thread(target=self.search_for_qr)
        self.qr_search_thread.start()

    def start_search_for_qr_door(self):
        with self.state_lock:
            if self.state != 'SEARCHING_DOOR_QR':
                return
            if self.qr_search_active:
                return
        self.qr_search_thread = threading.Thread(target=self.search_for_qr_door)
        self.qr_search_thread.start()

    def stop_movement(self):
        # Остановка движения дрона
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0  # Убедитесь, что вертикальная скорость равна нулю
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.velocity_pub.publish(twist)

    def stop_drone(self):
        self.get_logger().info('Остановка дрона из-за обнаружения QR-кода.')
        # Логика остановки дрона, например, установка текущей позиции как целевой
        pose = PoseStamped()
        pose.header = Header()
        pose.header.stamp = self.get_clock().now().to_msg()

        # Поддерживаем текущую высоту
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 2.5

        self.local_pos_pub.publish(pose)

    def test_services(self):
        # Ждем, пока дрон не подключится
        while not self.current_state or not self.current_state.connected:
            rclpy.spin_once(self)

        # Проверка доступности сервиса армирования
        if self.arming_client.wait_for_service(timeout_sec=5.0):
            arm_req = CommandBool.Request()
            arm_req.value = True
            future = self.arming_client.call_async(arm_req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() and future.result().success:
                self.get_logger().info('Дрон вооружен')
            else:
                self.get_logger().error('Не удалось вооружить дрон')
        else:
            self.get_logger().error('Сервис армирования недоступен')

        # Установка режима OFFBOARD
        if self.set_mode_client.wait_for_service(timeout_sec=5.0):
            set_mode_req = SetMode.Request()
            set_mode_req.custom_mode = 'OFFBOARD'
            future = self.set_mode_client.call_async(set_mode_req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() and future.result().mode_sent:
                self.get_logger().info('Режим OFFBOARD установлен')
            else:
                self.get_logger().error('Не удалось установить режим OFFBOARD')
        else:
            self.get_logger().error('Сервис установки режима недоступен')

        # Взлет
        self.takeoff()

        # Запуск поиска QR-кода на полу после взлёта
        with self.state_lock:
            self.state = 'SEARCHING_FLOOR_QR'  # Переход в состояние поиска QR на полу
        self.start_search_for_qr()

    def takeoff(self):
        self.get_logger().info('Попытка взлета...')

        # Создаем сообщение PoseStamped для взлета
        pose = PoseStamped()
        pose.header = Header()
        pose.header.stamp = self.get_clock().now().to_msg()

        # Устанавливаем целевую высоту (например, 2.5 метра)
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 2.4

        # Публикуем целевую позицию несколько раз для взлета
        for _ in range(0, 100):
            if self.qr_data_floor:  # Проверяем, обнаружен ли QR-код на полу
                break  # Останавливаем цикл, если QR-код на полу обнаружен
            self.local_pos_pub.publish(pose)
            rclpy.spin_once(self)
            time.sleep(0.1)

        self.get_logger().info('Команда на взлет отправлена. Проверьте, достиг ли дрон целевой высоты.')

        # Публикуем целевую позицию несколько раз для движения вперед
        for _ in range(0, 20):
            self.local_pos_pub.publish(pose)
            rclpy.spin_once(self)
            time.sleep(0.1)

    def land(self):
        self.get_logger().info('Посадка...')

        # Устанавливаем режим на посадку
        if self.set_mode_client.wait_for_service(timeout_sec=5.0):
            set_mode_req = SetMode.Request()
            set_mode_req.custom_mode = 'AUTO.LAND'
            future = self.set_mode_client.call_async(set_mode_req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() and future.result().mode_sent:
                self.get_logger().info('Режим посадки установлен')
            else:
                self.get_logger().error('Не удалось установить режим посадки')
        else:
            self.get_logger().error('Сервис установки режима недоступен')

def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
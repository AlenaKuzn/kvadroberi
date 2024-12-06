#Дополнение к p14.py, поиск QR-кода и облет комнаты и добавление лидара при обнаружении qr садится
#Добавление поиска QR-кода над дверью со второй камеры, того qr с пола
#Добавление остановки движения при обнаружении qr над дверью и приближение к нему 
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

        # Создаем клиента для установки режима
        self.set_mode_client = self.create_client(SetMode, '/uav1/mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, '/uav1/mavros/cmd/arming')

        # Публишер для установки целевой позиции дрона (для взлета)
        self.local_pos_pub = self.create_publisher(PoseStamped, '/uav1/mavros/setpoint_position/local', 10)

        # Подписываемся на топик состояния дрона (для проверки состояния)
        self.state_sub = self.create_subscription(State, '/uav1/mavros/state', self.state_cb, 10)

        # Текущее состояние дрона
        self.current_state = None

        # Подписываемся на топик нижней камеры
        self.camera_sub = self.create_subscription(Image, '/uav1/camera_down', self.camera_cb, 10)
        self.bridge = CvBridge()

        self.qr_data_floor = None  # Переменная для хранения данных QR-кода на полу
        self.qr_data_door = None   # Переменная для хранения данных QR-кода над дверью

        # Публишер для управления угловой скоростью
        self.velocity_pub = self.create_publisher(Twist, '/uav1/mavros/setpoint_velocity/cmd_vel_unstamped', 10)

        # Подписываемся на топик лидара
        self.lidar_sub = self.create_subscription(LaserScan, '/uav1/scan', self.lidar_cb, 10)
        self.min_distance = 0.5  # Минимальное безопасное расстояние до препятствия

        # Подписывамся на топик второй камеры
        self.camera_sub_2 = self.create_subscription(Image, '/uav1/camera', self.camera_cb_2, 10)

        self.qr_search_thread = None
        self.qr_search_active = False

        self.test_services()

    def state_cb(self, msg):
        self.current_state = msg

    def camera_cb(self, msg):
        # Обрабатываем изображение с камеры на полу
        image = self.convert_image(msg)
        qr_code_data, qr_position = self.detect_qr_code(image)
        if qr_code_data:
            self.get_logger().info(f'QR-код на полу найден: {qr_code_data}')
            self.qr_data_floor = qr_code_data
            self.stop_drone()  # Останавливаем дрон, но не садимся
            self.search_for_qr_door()  # Начинаем поиск QR-кода над дверью

    def camera_cb_2(self, msg):
        # Конвертируем ROS Image сообщение в OpenCV изображение
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Отображаем изоражение с передней камеры
        #cv2.imshow("Front Camera View", cv_image)
        #cv2.waitKey(1)  # Обновляем окно

        # Обрабатываем изображение с камеры над дверью
        if not self.qr_data_floor:
            return  # Не начинаем поиск, если QR-код на полу не найден

        #self.get_logger().info('Изображение с передней камеры обработано и отображено.')

        qr_code_data, qr_position = self.detect_qr_code(cv_image)
        if qr_code_data:
            self.get_logger().info(f'QR-код над дверью найден: {qr_code_data}')
            self.qr_data_door = qr_code_data
            self.approach_qr_door(qr_position)  # Подлетаем к QR-коду над дверью
            self.stop_movement()  # Останавливаем движение, если QR-код найден
            self.qr_search_active = False  # Останавливаем поток поиска

        # Запускаем поток поиска, если он еще не активен
        if not self.qr_data_door and not self.qr_search_active:
            self.qr_search_active = True
            self.qr_search_thread = threading.Thread(target=self.search_for_qr_door)
            self.qr_search_thread.start()

    def convert_image(self, msg):
        # Конвертируем сообщение изображения в формат, пригодный для обработки
        return cv2.cvtColor(self.bridge.imgmsg_to_cv2(msg, "bgr8"), cv2.COLOR_BGR2GRAY)

    def detect_qr_code(self, image):
        # Логика обнаружения QR-кода
        decoded_objects = pyzbar.decode(image)
        for obj in decoded_objects:
            # Получаем позицию QR-кода
            qr_position = obj.rect
            return obj.data.decode('utf-8'), qr_position
        return None, None

    def approach_qr(self, qr_position):
        # Логика подлета к QR-коду
        self.get_logger().info('Подлетаем ближе к QR-коду.')
        twist = Twist()
        # Пимер логики: если QR-код не в центре, корректируем позицию
        if qr_position:
            # Простейшая логика для подлета
            twist.linear.x = 0.2  # Подлетаем вперед
            twist.angular.z = 0.0  # Держим направление
            self.velocity_pub.publish(twist)
            time.sleep(2)  # Подлетаем в течение 2 секунд
        self.stop_movement()

    def stop_movement(self):
        # Останавливаем движение дрона
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.z = 0.0  # Убедитесь, что вертикальная скорость равна нулю
        twist.angular.z = 0.0
        self.velocity_pub.publish(twist)

    def lidar_cb(self, msg):
        # Обрабатываем данные лидара
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

        while not self.qr_data_floor:  # Пока QR-код на полу не обнаружен
            self.get_logger().info('Поиск QR-кода на полу продолжается...')
            # Поиск по спирали
            twist.linear.x = linear_speed
            twist.angular.y = angular_speed  # поменяла z на у
            self.velocity_pub.publish(twist)
            rclpy.spin_once(self)
            time.sleep(0.1)

        self.get_logger().info('QR-код на полу найден, остановка движения.')
        self.stop_movement()
        self.search_for_qr_door()  # Начинаем поиск QR-кода над дверью

    def search_for_qr_door(self):
        self.get_logger().info('Начало поиска QR-кода над дверью...')
        twist = Twist()
        angular_speed = 0.5  # Скорость вращения

        while self.qr_search_active and not self.qr_data_door:
            # Вращаем дрон
            twist.linear.x = 0.0
            twist.angular.z = angular_speed
            self.velocity_pub.publish(twist)
            time.sleep(0.1)

        self.stop_movement()
        self.qr_search_active = False
        self.get_logger().info('Поиск QR-кода завершен.')

    def approach_qr_door(self, qr_position):
        # Логика подлета к QR-коду над дверью
        self.get_logger().info('Подлетаем ближе к QR-коду над дверью.')
        twist = Twist()
        if qr_position:
            # Простейшая логика для подлета
            twist.linear.x = 0.2  # Подлетаем вперед
            twist.angular.z = 0.0  # Держим направление
            self.velocity_pub.publish(twist)
            time.sleep(2)  # Подлетаем в течение 2 секунд

        # Пролет через дверной проем
        self.fly_through_door()

    def fly_through_door(self):
        self.get_logger().info('Пролетаем через дверной проем.')
        twist = Twist()
        twist.linear.x = 0.5  # Увеличиваем скорость для пролета
        twist.angular.z = 0.0
        self.velocity_pub.publish(twist)
        time.sleep(3)  # Время, необходимое для пролета через дверной проем

        # Проверяем, что дрон в другой комнате
        if self.is_in_new_room():
            self.get_logger().info('Дрон в новой комнате, останавливаемся и начинаем поиск QR-кода на полу.')
            self.stop_movement()  # Останавливаем дрон
            self.qr_search_active = False  # Останавливаем поиск QR-кода над дверью
            self.qr_data_door = None  # Сбрасываем данные QR-кода над дверью
            self.qr_data_floor = None  # Сбрасываем данные QR-кода на полу
            self.search_for_qr()  # Начинаем поиск QR-кода на полу

    def is_in_new_room(self):
        # Пример логики для определения, что дрон в новой комнате
        # Используем данные лидара для проверки увеличения расстояния до ближайших объектов
        # Это упрощенный пример, в реальности может потребоваться более сложная логика
        min_range = min(self.lidar_ranges)
        return min_range > 2.0  # Если расстояние больше 2 метров, считаем, что дрон в новой комнате

    def test_services(self):
        # Ждем, пока дрон не подключится
        while not self.current_state or not self.current_state.connected:
            rclpy.spin_once(self)

        # Проверка доступности сервиса армирования
        if self.arming_client.wait_for_service(timeout_sec=1.0):
            arm_req = CommandBool.Request()
            arm_req.value = True
            future = self.arming_client.call_async(arm_req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() and future.result().success:
                self.get_logger().info('Дрон вооружен')
            else:
                self.get_logger().error('Не удалоь вооружить дрон')
        else:
            self.get_logger().error('Сервис армирования недоступен')

        # Установка режима OFFBOARD
        if self.set_mode_client.wait_for_service(timeout_sec=1.0):
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

        # Начинаем поиск QR-кода на полу
        self.search_for_qr()

        # Посадка
        #self.land()

    def takeoff(self):
        self.get_logger().info('Попытка взлета...')

        # Создаем сообщение PoseStamped для взлета
        pose = PoseStamped()
        pose.header = Header()
        pose.header.stamp = self.get_clock().now().to_msg()

        # Усанавливаем целевую высоту (например, 3 метра)
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 3.0

        # Публикуем целевую позицию несколько раз для взлета
        for _ in range(0, 100):
            if self.qr_data_floor:  # Проверяем, обнаружен ли QR-код на полу
                break  # Останавливаем цикл, если QR-код на полу обнаружен
            self.local_pos_pub.publish(pose)
            rclpy.spin_once(self)
            time.sleep(0.1)

        self.get_logger().info('Команда на взлет отправлена. Проверьте, достиг ли дрон целевой высоты.')


        # Публикуем целевую позицию несколько раз для движения вперед
        for _ in range(0, 100):
            self.local_pos_pub.publish(pose)
            rclpy.spin_once(self)
            time.sleep(0.1)

    def land(self):
        self.get_logger().info('Посадка...')

        # Устанавливаем режим на посаду
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

    def stop_drone(self):
        self.get_logger().info('Остановка дрона из-за обнаружения QR-кода.')
        # Логика остановки дрона, например, установка текущей позиции как целевой
        pose = PoseStamped()
        pose.header = Header()
        pose.header.stamp = self.get_clock().now().to_msg()
        #pose.pose.position.x = 0.0
        #pose.pose.position.y = 0.0
        pose.pose.position.z = 2.5  # Поддерживаем текущую высоту
        self.local_pos_pub.publish(pose)


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

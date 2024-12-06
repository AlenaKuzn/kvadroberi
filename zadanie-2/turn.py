import rclpy
from rclpy.node import Node
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import State, PositionTarget
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header
import time
import math

class DroneControlNode(Node):
    def __init__(self):
        super().__init__('drone_control_node')

        # Клиенты и паблишеры
        self.set_mode_client = self.create_client(SetMode, '/uav1/mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, '/uav1/mavros/cmd/arming')

        self.local_pos_pub = self.create_publisher(PositionTarget, '/uav1/mavros/setpoint_raw/local', 10)
        self.state_sub = self.create_subscription(State, '/uav1/mavros/state', self.state_cb, 10)

        self.current_state = None
        self.yaw_angle = 0.0  # Начальный yaw (курс) дрона
        self.turn_count = 0    # Счетчик поворотов
        self.max_turns = 4     # Нужно 4 поворота на 90 градусов

        # Армирование и взлет
        self.arm_and_takeoff(3.0)

    def state_cb(self, msg):
        """Обработчик состояния дрона."""
        self.current_state = msg

    def arm_and_takeoff(self, altitude):
        """Армирование и взлет."""
        self.wait_for_connection()
        self.arm_drone()
        self.set_offboard_mode()

        time.sleep(2)  # Задержка перед взлетом
        self.takeoff(altitude)
        time.sleep(5)  # Стабилизация на высоте

        self.perform_turns()  # Выполняем 4 поворота

        self.land()  # Посадка

    def wait_for_connection(self):
        """Ожидание подключения к FCU."""
        while not self.current_state or not self.current_state.connected:
            self.get_logger().info('Waiting for FCU connection...')
            rclpy.spin_once(self)

    def arm_drone(self):
        """Армирование дрона."""
        if self.arming_client.wait_for_service(timeout_sec=5.0):
            arm_req = CommandBool.Request()
            arm_req.value = True
            future = self.arming_client.call_async(arm_req)
            rclpy.spin_until_future_complete(self, future)
            self.get_logger().info('Drone armed!')

    def set_offboard_mode(self):
        """Установка режима Offboard."""
        if self.set_mode_client.wait_for_service(timeout_sec=5.0):
            set_mode_req = SetMode.Request()
            set_mode_req.custom_mode = 'OFFBOARD'
            future = self.set_mode_client.call_async(set_mode_req)
            rclpy.spin_until_future_complete(self, future)
            self.get_logger().info('OFFBOARD mode set!')

    def takeoff(self, altitude):
        """Взлет на заданную высоту."""
        self.get_logger().info(f'Taking off to {altitude} meters...')
        pose = self.create_position_target(0.0, 0.0, altitude, self.yaw_angle)
        self.publish_position(pose)

    def perform_turns(self):
        """Выполнение 4 поворотов на 90 градусов."""
        self.get_logger().info('Starting 90-degree turns...')
        for i in range(self.max_turns):
            self.yaw_angle += math.radians(90)  # Увеличиваем курс на 90 градусов
            self.get_logger().info(f'Turn {i + 1}/{self.max_turns}: Setting yaw to {math.degrees(self.yaw_angle)} degrees.')
            
            pose = self.create_position_target(0.0, 0.0, 3.0, self.yaw_angle)  # Высота остается той же
            self.publish_position(pose)

            time.sleep(5)  # Даем время для завершения поворота

    def create_position_target(self, x, y, z, yaw):
        """Создание цели для полета с заданным yaw."""
        target = PositionTarget()
        target.header = Header()
        target.header.stamp = self.get_clock().now().to_msg()
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        # Игнорируем все, кроме позиции и yaw
        target.type_mask = (
            PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )

        target.position.x = x
        target.position.y = y
        target.position.z = z

        # Задаем ориентацию по yaw
        target.yaw = yaw

        return target

    def publish_position(self, target):
        """Публикация целевой позиции."""
        for _ in range(10):
            self.local_pos_pub.publish(target)
            rclpy.spin_once(self)
            time.sleep(0.05)

    def land(self):
        """Посадка дрона."""
        self.get_logger().info('Landing...')
        if self.set_mode_client.wait_for_service(timeout_sec=5.0):
            set_mode_req = SetMode.Request()
            set_mode_req.custom_mode = 'AUTO.LAND'
            future = self.set_mode_client.call_async(set_mode_req)
            rclpy.spin_until_future_complete(self, future)

def main(args=None):
    rclpy.init(args=args)
    node = DroneControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


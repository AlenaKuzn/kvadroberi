import rclpy
from rclpy.node import Node
from drone_control_node import DroneControlNode
from navigation import RoomNavigationNode
import time

class MainController(Node):
    def __init__(self):
        super().__init__('main_controller')

        # Инициализируем узлы управления дроном и навигации
        self.drone_node = DroneControlNode()
        self.navigation_node = RoomNavigationNode()

        # Запуск дрона и навигации
        self.start_drone_and_navigation()

    def start_drone_and_navigation(self):
        """Армирование дрона, установка режима Offboard, взлёт и навигация."""
        self.get_logger().info('Arming drone...')
        self.drone_node.arm_drone()
        self.get_logger().info('Setting OFFBOARD mode...')
        self.drone_node.set_offboard_mode()

        time.sleep(2)  # Пауза для стабилизации
        self.get_logger().info('Taking off...')
        self.drone_node.takeoff(2.0)
        time.sleep(5)  # Ожидание взлёта

        # Запуск цикла навигации по комнатам
        self.run_navigation()

    def run_navigation(self):
        """Цикл навигации дрона с использованием лидара."""
        self.get_logger().info('Starting room navigation...')
        
        try:
            rclpy.spin(self.navigation_node)  # Запуск навигационного узла
        except KeyboardInterrupt:
            self.get_logger().info('Navigation interrupted. Landing drone...')
            self.drone_node.land()
        finally:
            self.shutdown()

    def shutdown(self):
        """Остановка узлов и завершение работы."""
        self.navigation_node.destroy_node()
        self.drone_node.destroy_node()
        self.destroy_node()  # Остановка MainController
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    controller = MainController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down...')
    finally:
        controller.shutdown()

if __name__ == '__main__':
    main()


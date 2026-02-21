#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Car(Node):
    """
    Узел-подписчик, который получает новости робота.
    """
    
    def __init__(self):
        super().__init__('car')
        
        # Создаём Subscriber
        self.subscriber_ = self.create_subscription(
            String,                    # тип сообщения
            'traffic_light',              # имя топика
            self.callback_light,        # функция обработки
            10                         # размер очереди
        )
        
        self.get_logger().info('Car готов принимать трафик!')
    
    def callback_light(self, msg):
        """
        Эта функция вызывается при получении каждого сообщения.
        """
        if msg.data == "GREEN":
            self.get_logger().info(f'Еду!')
        elif msg.data == "YELLOW":
            self.get_logger().info(f'Замедляюсь...')
        else:
            self.get_logger().info(f'Остановка!')


def main(args=None):
    rclpy.init(args=args)
    node = Car()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
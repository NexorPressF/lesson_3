#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Импортируем тип сообщения


class TrafficLightNode(Node):
    """
    Узел-издатель, который публикует новости робота.
    """
    
    def __init__(self):
        super().__init__('traffic_light')
        
        # Создаём Publisher
        self.publisher_ = self.create_publisher(
            String,           # тип сообщения
            'traffic_light',     # имя топика
            10                # размер очереди
        )
        
        # Таймер для публикации каждые 0.5 секунды
        self.timer = self.create_timer(3, self.send_light)
        
        # Счётчик сообщений
        self.counter = 1
        
        self.get_logger().info('Robot News Station запущена!')
    
    def send_light(self):
        """
        Функция публикации новостей.
        """
        # Создаём сообщение
        msg = String()
        if self.counter == 1:
            msg.data = f'RED'
        elif self.counter == 2:
            msg.data = f"YELLOW"
        else:
            msg.data = f"GREEN"
            self.counter = 0
        
        # Публикуем сообщение
        self.publisher_.publish(msg)
        
        # Выводим в лог
        self.get_logger().info(f'Публикую: "{msg.data}"')
        
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
# echo-client.py

import socket
import time
import rclpy
from rclpy.node import Node
import irayple.msg

HOST = "192.168.0.8"  # The server's hostname or IP address
PORT = 3000  # The port used by the server


class Parser(Node):
    def __init__(self):
        super().__init__('irayple_parser')
        self.publisher = self.create_publisher(
            irayple.msg.Irayple,
            'irayple',
            10
        )
        self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        """
        Создает новое соединение каждый раз.
        Берет данные с камеры и публикует в топик.
        """
        try:
            # Создаем новое соединение каждый раз
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(1.0)  # Таймаут подключения
                s.connect((HOST, PORT))
                data = s.recv(1024)

                if data:
                    text = data.decode('utf-8').strip('()')
                    # Данные приходят в формате (x;y;z)
                    values = tuple(map(int, text.split(';')))
                    if len(values) >= 4:
                        x = values[0]
                        y = values[1]
                        information = values[2]
                        byte = str(values[3])  # Преобразуем в строку
                        # self.get_logger().info(f"Received: {values}")
                        self.publish_data(x, y, information, byte)
                    else:
                        self.get_logger().warn(f'Invalid data format')
        except socket.timeout:
            # Таймаут
            pass
        except ConnectionRefusedError:
            self.get_logger().warn(f'Connection {HOST}:{PORT}')
        except Exception as e:
            # self.get_logger().error(f'Error: {e}')
            pass

    def publish_data(self, x, y, information, byte):
        """Публикует данные."""
        try:
            msg = irayple.msg.Irayple()
            msg.x = int(x)
            msg.y = int(y)
            msg.information = int(information)
            # Убеждаемся что byte это строка
            msg.byte = str(byte) if byte is not None else ''
            # self.get_logger().info(f"Published: {msg}")
            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing data: {e}')


def main(args=None):
    """main."""
    rclpy.init(args=args)
    node = Parser()
    # задержка для инициализации
    time.sleep(0.5)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

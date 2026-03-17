#!/usr/bin/env python3
# Парсер камеры irayple. Данные: (x;y;v3;v4) или несколько подряд в одном пакете.

import re
import socket
import time
import rclpy
from rclpy.node import Node
import irayple.msg

HOST = "192.168.0.14"
PORT = 3000
LOCAL_IP = "192.168.0.10"


class Parser(Node):
    def __init__(self):
        super().__init__('irayple_parser')
        self.publisher = self.create_publisher(
            irayple.msg.Irayple,
            'irayple',
            10
        )
        self._last_conn_warn = 0.0
        self.create_timer(0.1, self.timer_callback)

    def _parse_packet(self, data):
        """возвращает список (x, y, information, byte)."""
        try:
            text = data.decode('utf-8', errors='ignore')
        except Exception:
            return []
        out = []
        for m in re.findall(r'\((-?\d+);(-?\d+);(\d+);(\d+)\)', text):
            out.append((int(m[0]), int(m[1]), int(m[2]), m[3]))
        return out

    def timer_callback(self):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(1.0)
                s.bind((LOCAL_IP, 0))
                s.connect((HOST, PORT))
                data = s.recv(4096)
                if data:
                    readings = self._parse_packet(data)
                    for x, y, information, byte in readings:
                        self.publish_data(x, y, information, byte)
        except socket.timeout:
            pass
        except (ConnectionRefusedError, OSError):
            t = time.time()
            if t - self._last_conn_warn >= 5.0:
                self.get_logger().warn(f'Connection {HOST}:{PORT} refused')
                self._last_conn_warn = t
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

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

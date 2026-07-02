#!/usr/bin/env python3
# Парсер камеры irayple. Данные: (x;y;v3;v4) или несколько подряд в одном пакете.

import re
import argparse
import socket
import time
import rclpy
from rclpy.node import Node
import irayple.msg

DEFAULT_HOST, DEFAULT_PORT = '192.168.49.14', 3000
LOCAL_IP = '192.168.49.31'
TOPIC = 'irayple'


class Parser(Node):
    def __init__(self, host=None, port=None, localip=None, topic=None):
        topic_for_name = topic if topic is not None else TOPIC
        safe_postfix = topic_for_name.replace('/', '_').replace('-', '_')
        super().__init__(f'irayple_parser_{safe_postfix}')
        self.declare_parameter('host', DEFAULT_HOST)
        self.declare_parameter('port', DEFAULT_PORT)
        self.declare_parameter('localip', LOCAL_IP)
        self.declare_parameter('topic', TOPIC)
        self.host = host if host is not None else self.get_parameter('host').value
        self.port = (
            port if port is not None
            else int(self.get_parameter('port').value)
        )
        self.localip = (
            localip if localip is not None
            else self.get_parameter('localip').value
        )
        self.topic = (
            topic if topic is not None
            else self.get_parameter('topic').value
        )
        self.publisher = self.create_publisher(
            irayple.msg.Irayple,
            self.topic,
            10
        )
        self._last_conn_warn = 0.0
        self.socket = None
        self.buffer = b''
        self.create_timer(1.0, self._check_connection)
        self.create_timer(0.05, self.timer_callback)

    def _check_connection(self):
        if not self.socket or self.socket.fileno() == -1:
            try:
                self._connect()
            except Exception as e:
                self.get_logger().error(f'Error connecting to camera: {e}')
                return False
        return True

    def _connect(self):
        if self.socket:
            try:
                self.socket.close()
            except Exception:
                pass
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.settimeout(1.0)
        self.socket.bind((self.localip, 0))
        self.socket.connect((self.host, self.port))

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
            if not self._check_connection():
                return
            self.socket.settimeout(1.0)
            data = self.socket.recv(4096)
            if data:
                readings = self._parse_packet(data)
                for x, y, information, byte in readings:
                    self.publish_data(x, y, information, byte)
        except socket.timeout:
            pass
        except (ConnectionRefusedError, OSError):
            t = time.time()
            if t - self._last_conn_warn >= 5.0:
                self.get_logger().warn(f'Camera connection refused')
                self._last_conn_warn = time.time()
            try:
                self._connect()
            except Exception:
                pass
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            try:
                self._connect()
            except Exception:
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
    import sys

    rclpy.init(args=args)
    kwargs = {}
    if any(
        flag in sys.argv
        for flag in ('--host', '--port', '--localip', '--topic')
    ):
        ap = argparse.ArgumentParser()
        ap.add_argument('--host', default=DEFAULT_HOST)
        ap.add_argument('--port', type=int, default=DEFAULT_PORT)
        ap.add_argument('--localip', default=LOCAL_IP)
        ap.add_argument('--topic', default=TOPIC)
        cli, _ = ap.parse_known_args()
        kwargs = {
            'host': cli.host,
            'port': cli.port,
            'localip': cli.localip,
            'topic': cli.topic,
        }
    node = Parser(**kwargs)
    # задержка для инициализации
    time.sleep(0.5)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""При отрисовке создает файл с координатами маркеров."""
import math
import os

import rclpy
import argparse
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from tf2_ros import Buffer, TransformListener, TransformException
from irayple.msg import Irayple

IRAYPLE_TOPIC = '/irayple'


def angle(angl):
    """Приводит угол к диапазону [0, 2*pi]."""
    while angl > 2 * math.pi:
        angl -= 2 * math.pi
    while angl < 0:
        angl += 2 * math.pi
    return angl


class IraypleSlam(Node):
    """При отрисовке создает файл с координатами маркеров."""

    def __init__(self, irayple_topic=IRAYPLE_TOPIC):
        """Инит."""
        super().__init__('irayple_slam')

        # ищем путь к файлу с координатами маркеров
        package_share = get_package_share_directory('irayple')
        self.path = os.path.join(package_share, 'pose', 'markers_config.txt')

        # фреймы для нахождения трансформации мап -> бейс футпринт
        self.declare_parameter('irayple_topic', IRAYPLE_TOPIC)
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.base_link_frame = self.declare_parameter(
            'base_frame', 'base_footprint'
        ).value

        # масштабирование
        self.irayple_scale = self.declare_parameter(
            'irayple_scale', 0.0001
        ).value
        self.info_scale_deg = self.declare_parameter(
            'information_scale_deg', 0.1
        ).value
        self.flip_180 = int(self.declare_parameter('flip_180', 0).value)

        # буфер трансформации
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # подписка на топик с маркерами
        self.create_subscription(
            Irayple,
            self.get_parameter('irayple_topic').value,
            self._cb,
            30
        )

        # координаты маркеров локально (для этого скрипта)
        self._markers = {}
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        # записываем шапку в файл
        with open(self.path, 'w', encoding='utf-8') as f:
            f.write('# byte x y yaw(radians) flip_180 (все через 1 пробел)\n')
            f.write('#\n')

    def _slam_callback(self):
        """Обновляет TF map->base с учетом цепочки через odom."""
        try:
            tr = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_link_frame,
                rclpy.time.Time()
            )
        except TransformException:
            return
        self._x = tr.transform.translation.x
        self._y = tr.transform.translation.y
        self._yaw = 2.0 * math.atan2(
            tr.transform.rotation.z,
            tr.transform.rotation.w
        )
        # print(f'x: {self._x}, y: {self._y}, yaw: {self._yaw}')

    def _write_markers(self, marker_id, x, y, yaw):
        """Записывает координаты маркеров в файл."""
        with open(self.path, 'a', encoding='utf-8') as f:
            f.write(str(marker_id) + ' ')
            f.write(str(x) + ' ')
            f.write(str(y) + ' ')
            f.write(str(yaw) + ' ')
            f.write(str(0) + '\n')
        print(f'маркер {marker_id}записан')

    def _cb(self, msg: Irayple):
        """Колбек для чтения маркеров."""
        # если позиция не получена, не записываем
        self._slam_callback()
        if self._x is None or self._y is None or self._yaw is None:
            return
        marker_id = msg.byte
        # если маркера нет в локальном словаре, записываем его в файл и словарь
        if marker_id not in self._markers.keys():
            if self.flip_180 == 1:
                marker_x = self._x + msg.y * self.irayple_scale
                marker_y = self._y - msg.x * self.irayple_scale
            else:
                marker_x = self._x - msg.y * self.irayple_scale
                marker_y = self._y + msg.x * self.irayple_scale

            marker_yaw = self._yaw + (
                float(msg.information) * self.info_scale_deg * math.pi / 180.0
            )

            # приводим угол к диапазону [0, 2*pi]
            marker_yaw = angle(marker_yaw)

            # записываем маркер в файл и словарь
            self._write_markers(marker_id, marker_x, marker_y, marker_yaw)
            self._markers[marker_id] = [
                marker_x, marker_y, marker_yaw, self.flip_180
            ]


def main(args=None):
    """Мейн."""
    import sys
    kwargs = {}
    if '--irayple_topic' in sys.argv:
        parser = argparse.ArgumentParser()
        parser.add_argument('--irayple_topic', default=IRAYPLE_TOPIC)
        cli = parser.parse_args()
        kwargs = {
            'irayple_topic': cli.irayple_topic,
        }

    rclpy.init(args=args)
    node = IraypleSlam(**kwargs)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

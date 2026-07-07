#!/usr/bin/env python3
"""При отрисовке создает файл с координатами маркеров."""
import math
import os

import rclpy
import argparse
from rclpy.node import Node
from irayple.markers_config import markers_config_path
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

        self.path = markers_config_path()
        os.makedirs(os.path.dirname(self.path), exist_ok=True)

        self.declare_parameter('irayple_topic', IRAYPLE_TOPIC)
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.base_link_frame = self.declare_parameter(
            'base_frame', 'base_footprint'
        ).value

        self.irayple_scale = self.declare_parameter(
            'irayple_scale', 0.0001
        ).value
        self.info_scale_deg = self.declare_parameter(
            'information_scale_deg', 0.1
        ).value
        self.flip_180 = int(self.declare_parameter('flip_180', 0).value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            Irayple,
            self.get_parameter('irayple_topic').value,
            self._cb,
            30
        )

        self._markers = {}
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        with open(self.path, 'w', encoding='utf-8') as f:
            f.write('# byte x y yaw(radians) flip_180 (все через 1 пробел)\n')
            f.write('#\n')
        self.get_logger().info(f'Запись маркеров в {self.path}')

    def _slam_callback(self):
        """Обновляет TF map->base с учетом цепочки через odom."""
        try:
            tr = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_link_frame,
                rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().warn(
                f'Нет TF {self.map_frame}->{self.base_link_frame}: {ex}',
                throttle_duration_sec=5.0,
            )
            return
        self._x = tr.transform.translation.x
        self._y = tr.transform.translation.y
        self._yaw = 2.0 * math.atan2(
            tr.transform.rotation.z,
            tr.transform.rotation.w
        )

    def _write_markers(self, marker_id, x, y, yaw):
        """Записывает координаты маркеров в файл."""
        with open(self.path, 'a', encoding='utf-8') as f:
            f.write(str(marker_id) + ' ')
            f.write(str(x) + ' ')
            f.write(str(y) + ' ')
            f.write(str(yaw) + ' ')
            f.write(str(0) + '\n')
        self.get_logger().info(
            f'Маркер {marker_id} записан: x={x:.3f} y={y:.3f} yaw={yaw:.3f}'
        )

    def _cb(self, msg: Irayple):
        """Колбек для чтения маркеров."""
        self._slam_callback()
        marker_id = msg.byte
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
            marker_yaw = angle(marker_yaw)

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

#!/usr/bin/env python3
"""Publish qr pose to /qr_pose."""
import argparse
import math
import time

from geometry_msgs.msg import PoseWithCovarianceStamped

from irayple.markers_config import markers_config_path
from irayple.msg import Irayple

from nav_msgs.msg import Odometry

import rclpy
from rclpy.node import Node

import tf_transformations as tft

_IRAYPLE_SCALE = 0.0001  # мм -> м
_INFORMATION_SCALE_DEG = 0.1  # 0.1 -> 1 град

_ODOM_TOPIC = '/wheel_odom'
MAX_VEL = 0.5
IRAYPLE_TOPIC = '/irayple'
_QR_POSE_TOPIC = '/qr_pose'
_INITIALPOSE_TOPIC = '/initialpose'


class IraypleQrPosePub(Node):
    """Publish qr pose to /qr_pose."""

    def __init__(self, irayple_topic=IRAYPLE_TOPIC):
        """Init."""
        super().__init__('irayple_qr_pose_pub')

        self.path = markers_config_path()
        self.get_logger().info(f'Load from {self.path}')

        self.declare_parameter('irayple_topic', irayple_topic)
        self.declare_parameter('qr_pose_topic', _QR_POSE_TOPIC)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_initialpose', False)
        self.declare_parameter('initialpose_min_period_s', 1.0)

        self.irayple_scale = _IRAYPLE_SCALE
        self.info_scale_deg = _INFORMATION_SCALE_DEG

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.publish_initialpose = bool(
            self.get_parameter('publish_initialpose').value
        )
        self.initialpose_min_period_s = float(
            self.get_parameter('initialpose_min_period_s').value
        )

        self._pub_raw = self.create_publisher(
            PoseWithCovarianceStamped,
            _QR_POSE_TOPIC,
            30
        )
        self._pub_init = self.create_publisher(
            PoseWithCovarianceStamped,
            _INITIALPOSE_TOPIC,
            10
        )
        self.create_subscription(
            Irayple,
            self.get_parameter('irayple_topic').value,
            self._cb,
            30
        )

        # подписка на одом для получения скорости
        self.create_subscription(
            Odometry,
            _ODOM_TOPIC,
            self._odom_callback,
            10
        )

        self._last_stamp = 0.0
        self._last_msg = None
        self._markers = self._load_markers()
        self._last_initialpose_pub_time = 0.0

        # скорость
        self.speed = 0.0

    def _load_markers(self):
        markers = {}
        try:
            with open(self.path, 'r', encoding='utf-8') as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                    parts = line.split()
                    if len(parts) < 4:
                        self.get_logger().warn(f'Skip invalid marker: {line}')
                        continue
                    marker_id = parts[0].strip()
                    x = float(parts[1])
                    y = float(parts[2])
                    yaw_rad = float(parts[3])
                    markers[marker_id] = (x, y, yaw_rad)
        except Exception as e:
            self.get_logger().error(f'Failed to read config: {e}')
            return {}
        return markers

    def _get_raw_pose(self, msg):
        stamp = self.get_clock().now().to_msg()
        byte_key = str(msg.byte).strip()
        if byte_key not in self._markers:
            self.get_logger().warn(f'{byte_key} not in markers config.')
            return

        marker_x, marker_y, marker_yaw_rad = self._markers[byte_key]

        # information: 1-3599 (угол поворота камеры относительно маркера)
        # Смещение в осях камеры/робота: x вперёд, y влево (как в navigator).
        local_x = msg.x * self.irayple_scale
        local_y = msg.y * self.irayple_scale
        rel_yaw_rad = (
            -float(msg.information) * self.info_scale_deg * math.pi / 180.0
        )
        pose = (
            tft.translation_matrix((marker_x, marker_y, 0.0))
            @ tft.quaternion_matrix(
                tft.quaternion_from_euler(0.0, 0.0, marker_yaw_rad)
            )
            @ tft.translation_matrix((local_x, local_y, 0.0))
            @ tft.quaternion_matrix(
                tft.quaternion_from_euler(0.0, 0.0, rel_yaw_rad)
            )
        )
        x_m = float(pose[0, 3])
        y_m = float(pose[1, 3])
        qx, qy, qz, qw = tft.quaternion_from_matrix(pose)

        out_raw = PoseWithCovarianceStamped()
        out_raw.header.stamp = stamp
        out_raw.header.frame_id = self.frame_id
        out_raw.pose.pose.position.x = x_m
        out_raw.pose.pose.position.y = y_m
        out_raw.pose.pose.position.z = 0.0
        out_raw.pose.pose.orientation.x = float(qx)
        out_raw.pose.pose.orientation.y = float(qy)
        out_raw.pose.pose.orientation.z = float(qz)
        out_raw.pose.pose.orientation.w = float(qw)
        out_raw.pose.covariance = [
            1e-10, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1e-10, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e-10, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e-10, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e-10, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1e-12
        ]
        return out_raw

    def _odom_callback(self, msg: Odometry):
        self.speed = msg.twist.twist.linear.x

    def _cb(self, msg: Irayple):
        self._last_msg = msg
        out_raw = self._get_raw_pose(msg)
        if (out_raw is not None) and (self.speed < MAX_VEL):
            self._pub_raw.publish(out_raw)
            now_s = time.monotonic()
            if (
                self.publish_initialpose and
                (now_s - self._last_initialpose_pub_time) >=
                self.initialpose_min_period_s
            ):
                self._pub_init.publish(out_raw)
                self._last_initialpose_pub_time = now_s


def main(args=None):
    """Func main."""
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
    node = IraypleQrPosePub(**kwargs)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

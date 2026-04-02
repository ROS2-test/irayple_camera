#!/usr/bin/env python3
import math
import time
import os

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
from irayple.msg import Irayple


def _quat_from_yaw(yaw: float):
    """Кватернион из угла yaw. Точнее чем стандартные ф-ции."""
    yaw_rad = yaw * math.pi / 180.0
    half = 0.5 * yaw_rad
    return (0.0, 0.0, math.sin(half), math.cos(half))

class IraypleQrPosePub(Node):
    """Публикует позу по QR."""

    def __init__(self):
        super().__init__("irayple_qr_pose_pub")

        package_share = get_package_share_directory('irayple')
        self.path = os.path.join(package_share, "pose", "markers_config.txt")

        self.declare_parameter("irayple_topic", "/irayple")
        self.declare_parameter("qr_pose_topic", "/qr_pose")  
        self.declare_parameter("frame_id", "map")        
        self.declare_parameter("irayple_scale", 0.0001)  # мм -> м
        self.declare_parameter("information_scale_deg", 0.1)  # 0.1 град -> 1 град

        self.irayple_scale = float(self.get_parameter("irayple_scale").value)
        self.info_scale_deg = float(self.get_parameter("information_scale_deg").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        self._pub_raw = self.create_publisher(
            PoseWithCovarianceStamped,
            self.get_parameter("qr_pose_topic").value,
            30
        )
        self._pub_init = self.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose",
            10
        )
        self.create_subscription(
            Irayple,
            self.get_parameter("irayple_topic").value,
            self._cb,
            30
        )

        self._timer = self.create_timer(0.5, self._timer_callback)

        self._last_stamp = 0.0
        self._last_msg = None
        self._markers = self._load_markers()
        self._timer_flag = False
        
    def _load_markers(self):
        markers = {}
        try:
            with open(self.path, "r", encoding="utf-8") as f:
                for line in f:
                    if line.startswith("#"):
                        continue
                    else:
                        self.get_logger().info(line)
                        p1, p2, p3, p4 = line.strip().split()
                        print(p1, p2, p3, p4)
                        markers[str(p1).strip()] = (float(p2), float(p3), float(p4))
        except Exception as e:
            self.get_logger().error(f"Failed to read config: {e}")
            return {}
        return markers

    def _get_raw_pose(self, msg):
        stamp = self.get_clock().now().to_msg()
        byte_key = str(msg.byte).strip()
        if byte_key not in self._markers:
            self.get_logger().warn(f"byte '{byte_key}' not in markers config.")
            return
        # raw camera pose (x, y)
        x_m = - msg.y * self.irayple_scale + self._markers[byte_key][0]
        y_m = msg.x * self.irayple_scale + self._markers[byte_key][1]

        # information: 1-3599
        marker_yaw = self._markers[byte_key][2] * 180.0 / math.pi
        yaw_deg = -float(msg.information) * self.info_scale_deg + marker_yaw
        qx, qy, qz, qw = _quat_from_yaw(yaw_deg)

        out_raw = PoseWithCovarianceStamped()
        out_raw.header.stamp = stamp
        out_raw.header.frame_id = self.frame_id
        out_raw.pose.pose.position.x = float(x_m)
        out_raw.pose.pose.position.y = float(y_m)
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

    def _cb(self, msg: Irayple):
        self._last_msg = msg
        out_raw = self._get_raw_pose(msg)
        if out_raw is not None:
            self._pub_raw.publish(out_raw)
            if self._timer_flag:
                self._pub_init.publish(out_raw)
                self._timer_flag = False

    def _timer_callback(self):
        self._timer_flag = True

def main(args=None):
    rclpy.init(args=args)
    node = IraypleQrPosePub()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


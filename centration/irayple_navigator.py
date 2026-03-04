#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from irayple.msg import Irayple


def normalize_angle(angle):
    """Приводит угол от -пи до пи."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def normalize_angle90(angle):
    """Приводит угол к ближайшему из 0, 90, 180, 270."""
    angle = normalize_angle(angle)
    qater = round(angle / (0.5 * math.pi))
    return qater * (0.5 * math.pi)


def quaternion_to_yaw(q):
    """кватернион -> угол."""
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    return yaw


class IraypleNavigator(Node):
    """Нода для приезда к 0,0,0."""

    def __init__(self):
        """Инициализация."""
        super().__init__('irayple_navigator')
        self.max_ang_speed = 0.3
        self.max_linear_speed = 0.1
        self.dist_tolerance = 0.005  # м
        self.angle_tolerance = math.radians(3)  # град
        self.rate = 20
        self.MAX_ANGLE_DEG = 20.0  # макс угол поворота под столом
        self.irayple_scale = 0.001  # мм -> метры

        self._angle_integral = 0.0
        self._dist_integral = 0.0

        self.x, self.y, self.theta = 0.0, 0.0, 0.0

        # флаг получения данных с камеры
        self.get_data = False
        # флаг цель рассчитана один раз при первом qr
        self.calculated = False
        # флаг получены данные odom
        self.odom_received = False
        # флаг достижения цели
        self.done = False
        # True- центровка к 0, 90, 180, 270
        self.center_to_q = True

        self.subscription_irayple = self.create_subscription(
            Irayple,
            '/irayple',
            self.irayple_callback,
            10
        )

        self.subscription_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.publisher_cmd_vel = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        self.publisher_irayple_is_read = self.create_publisher(
            String,
            '/irayple_is_read',
            10
        )

        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

    def calculate_center_point(self):
        """Вычисление координат центра."""
        self.dx = self.irayple_x - self.odom_x
        self.dy = self.irayple_y - self.odom_y
        self.dtheta = self.irayple_theta - self.odom_theta

    def calculate_point(self):
        """
        Вычисляет точку, до которой робот должен отъехать назад,
        чтобы из неё доехать до 0, 0, 0.
        """
        self.d = (self.dy)/(math.cos(0.5 * math.pi - math.radians(
            self.MAX_ANGLE_DEG)))
        self.forward = self.dx + self.dx*math.tan(0.5 * math.pi - math.radians(
            self.MAX_ANGLE_DEG))

        self.xt = self.dx - self.d*math.sin(math.radians(self.MAX_ANGLE_DEG))
        self.yt = self.dy - self.d*math.cos(math.radians(self.MAX_ANGLE_DEG))

        return self.xt, self.yt, math.radians(self.MAX_ANGLE_DEG)

    def calculate_point_a(self):
        """Точка подъезда при 20 относительно 0, 90, 180, 270."""
        angle = normalize_angle(
            self.irayple_theta + math.radians(self.MAX_ANGLE_DEG))
        self.d = (self.dy) / (math.cos(0.5 * math.pi - angle))
        self.forward = self.dx + self.dx * math.tan(0.5 * math.pi - angle)
        self.xt = self.dx - self.d * math.sin(angle)
        self.yt = self.dy - self.d * math.cos(angle)
        return self.xt, self.yt, angle

    def pi_angle(self, tangle):
        """PI для угла."""
        kp, ki = 1.0, 0.1
        error = normalize_angle(tangle - self.odom_theta)
        dt = 1.0 / self.rate

        self._angle_integral += error * dt
        self._angle_integral = max(-1.0, min(1.0, self._angle_integral))

        out = kp * error + ki * self._angle_integral
        return max(-self.max_ang_speed, min(self.max_ang_speed, out))

    def pi_distance(self, tx, ty):
        """PI для расстояния."""
        kp, ki = 0.5, 0.05
        error = math.sqrt((tx - self.odom_x)**2 + (ty - self.odom_y)**2)
        dt = 1.0 / self.rate

        self._dist_integral += error * dt
        self._dist_integral = max(-0.5, min(0.5, self._dist_integral))

        out = kp * error + ki * self._dist_integral
        return max(-self.max_linear_speed, min(self.max_linear_speed, out))

    def reset_pi(self):
        """Сброс интеграла."""
        self._angle_integral = 0.0
        self._dist_integral = 0.0

    def odom_callback(self, msg):
        """Колбек для чтения odom."""
        self.odom_received = True
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_theta = quaternion_to_yaw(msg.pose.pose.orientation)

    def irayple_callback(self, msg):
        """Колбек чтения irayple."""
        self.get_data = True
        self.irayple_x = msg.x * self.irayple_scale
        self.irayple_y = msg.y * self.irayple_scale
        self.irayple_theta = math.radians(msg.information)
        self.irayple_byte = msg.byte

        # Рассчитываем цель один раз при первом виде qr
        if not self.calculated and self.odom_received:
            self.calculate_center_point()
            # выбираем: центр к 0 или 0, 90, 180, 270
            if self.center_to_q:
                self.irayple_theta = normalize_angle90(self.odom_theta)
                self.calculate_point_a()
            else:
                self.calculate_point()
            self.calculated = True
            self.get_logger().info(
                f'xt={self.xt:.4f}, yt={self.yt:.4f}'
            )

    def timer_callback(self):
        """Колбек для таймера."""
        # поднимаем флаг если данные с камеры полуены
        if self.get_data:
            self.publisher_irayple_is_read.publish(String(data='True'))
        else:
            self.publisher_irayple_is_read.publish(String(data='False'))

        # Едем к цели только если она уже рассчитана
        if not self.calculated:
            return
        if self.done:
            return

        while (
            abs(self.odom_x - self.xt) > 0.002 and
            abs(self.odom_y - self.yt) > 0.002 and
            abs(normalize_angle(self.odom_theta - self.irayple_theta)) > math.radians(2)):
            self.done = False

            # поворачиваемся на MAX_ANGLE_DEG относительно 0, 90, 180, 270
            angle = (
                normalize_angle(self.irayple_theta + math.radians(
                    self.MAX_ANGLE_DEG))
                if self.center_to_q else math.radians(self.MAX_ANGLE_DEG))
            while abs(normalize_angle(self.odom_theta-angle)) > math.radians(1):
                cmd = Twist()
                cmd.angular.z = self.pi_angle(angle)
                self.publisher_cmd_vel.publish(cmd)
                rclpy.spin_once(self, timeout_sec=0.05)

            # едем к точке xt, yt
            while math.hypot(self.xt - self.odom_x, self.yt - self.odom_y) > 0.002:
                cmd = Twist()
                cmd.linear.x = self.pi_distance(self.xt, self.yt)
                self.publisher_cmd_vel.publish(cmd)
                rclpy.spin_once(self, timeout_sec=0.05)

            # поворачиваемся на угол 0
            while abs(normalize_angle(self.odom_theta - self.irayple_theta)) > math.radians(1):
                cmd = Twist()
                cmd.angular.z = self.pi_angle(self.irayple_theta)
                self.publisher_cmd_vel.publish(cmd)

                rclpy.spin_once(self, timeout_sec=0.05)

            # едем к точке irayple_x, irayple_y
            while math.hypot(self.irayple_x - self.odom_x, self.irayple_y - self.odom_y) > 0.002:
                cmd = Twist()
                cmd.linear.x = self.pi_distance(self.irayple_x, self.irayple_y)
                self.publisher_cmd_vel.publish(cmd)
                rclpy.spin_once(self, timeout_sec=0.05)

            # поворачиваемся на угол 0
            while abs(normalize_angle(self.odom_theta - self.irayple_theta)) > math.radians(1):
                cmd = Twist()
                cmd.angular.z = self.pi_angle(self.irayple_theta)
                self.publisher_cmd_vel.publish(cmd)
                rclpy.spin_once(self, timeout_sec=0.05)

            # стоп
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.publisher_cmd_vel.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)

            self.done = True
            # ждем пока придет свежая одометрия и камерная одометрия
            self.odom_received = False
            self.get_data = False
            while not self.odom_received or not self.get_data:
                rclpy.spin_once(self, timeout_sec=0.05)
            self.odom_received = True
            self.get_data = True


def main(args=None):
    """Мейн."""
    rclpy.init(args=args)
    node = IraypleNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

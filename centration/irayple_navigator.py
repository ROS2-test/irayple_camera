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

        # Состояние алгоритма (не блокируем executor в timer_callback)
        # этапы: rotate1 -> drive2 -> rotate3 -> drive3 -> wait_updates -> done
        self._stage = 'ready'
        self._target_angle = 0.0
        self._target_x = 0.0
        self._target_y = 0.0

        # таймер для центрации
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
        kp, ki = 1.0, 0.1
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
            # гарантируем остановку
            cmd = Twist()
            self.publisher_cmd_vel.publish(cmd)
            return

        # запускаем
        if self._stage == 'ready':
            self._stage = 'rotate1'

        cmd = Twist()

        # угол для первого поворота
        if self._stage == 'rotate1':
            # если к ближайшему 0, 90, 180, 270, то center_to_q = True
            # если к 0, то center_to_q = False
            self._target_angle = (
                normalize_angle(self.irayple_theta + math.radians(self.MAX_ANGLE_DEG))
                if self.center_to_q else math.radians(self.MAX_ANGLE_DEG)
            )
            if abs(normalize_angle(self.odom_theta - self._target_angle)) > math.radians(1):
                cmd.angular.z = self.pi_angle(self._target_angle)
            else:
                self.reset_pi()
                self._stage = 'drive1'

        # Едем к точке xt, yt
        if self._stage == 'drive1':
            self._target_x, self._target_y = self.xt, self.yt
            if math.hypot(self._target_x - self.odom_x, self._target_y - self.odom_y) > 0.002:
                cmd.linear.x = self.pi_distance(self._target_x, self._target_y)
            else:
                self.reset_pi()
                self._stage = 'rotate2'

        # поворот на irayple_theta
        if self._stage == 'rotate2':
            self._target_angle = self.irayple_theta
            if abs(normalize_angle(self.odom_theta - self._target_angle)) > math.radians(1):
                cmd.angular.z = self.pi_angle(self._target_angle)
            else:
                self.reset_pi()
                self._stage = 'drive2'

        # Едем к точке irayple_x, irayple_y (0, 0)
        if self._stage == 'drive2':
            self._target_x, self._target_y = self.irayple_x, self.irayple_y
            if math.hypot(self._target_x - self.odom_x, self._target_y - self.odom_y) > 0.002:
                cmd.linear.x = self.pi_distance(self._target_x, self._target_y)
            else:
                self.reset_pi()
                self._stage = 'rotate3'

        # Финальный доворот к irayple_theta
        if self._stage == 'rotate3':
            self._target_angle = self.irayple_theta
            if abs(normalize_angle(self.odom_theta - self._target_angle)) > math.radians(1):
                cmd.angular.z = self.pi_angle(self._target_angle)
            else:
                self.reset_pi()
                self._stage = 'wait_updates'
                # ждём свежие сообщения
                self.odom_received = False
                self.get_data = False

        # Ждём, пока придут свежая одометрия и данные с камеры
        if self._stage == 'wait_updates':
            cmd = Twist()  # стоп, пока ждём
            if self.odom_received and self.get_data:
                # если доехали до 0, 0, то done = True
                # иначе переходим в ready и делаем заново
                if math.hypot(self._target_x - self.odom_x, self._target_y - self.odom_y) < 0.002:
                    self.done = True
                    self._stage = 'done'
                else:
                    self._stage = 'ready'

        self.publisher_cmd_vel.publish(cmd)


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

#!/usr/bin/env python3
"""Нода для центровки робота по QR-коду."""
import rclpy
from rclpy.node import Node
import time
import math
from std_msgs.msg import Bool
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


# def normalize_angle90(angle):
#     """Приводит угол к ближайшему из 0, 90, 180, 270."""
#     angle = normalize_angle(angle)
#     qater = round(angle / (0.5 * math.pi))
#     return qater * (0.5 * math.pi)


def quaternion_to_yaw(q):
    """кватернион -> угол."""
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    return yaw


def shortest_angle_error_deci(target_deci: float, current_deci: float):
    """Кратчайшая ошибка угла в 0.1 град."""
    return (target_deci - current_deci + 1800.0) % 3600.0 - 1800.0


class PID:
    """Класс ПИДа."""

    def __init__(
        self,
        kp,
        ki=0,
        kd=0,
        setpoint=0,
        output_limit=1.0,
        max_integral_error=None
    ):
        """Инициализация ПИДа."""
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.output_limit = output_limit
        self.max_integral_error = max_integral_error

        self.last_error = 0
        self.integral = 0
        self.last_time = time.time()

    def reset(self):
        """Сброс ПИДа."""
        self.last_error = 0
        self.integral = 0
        self.last_time = time.time()

    def update(self, current_value):
        """Обновление ПИДа."""
        now = time.time()
        dt = now - self.last_time
        if dt <= 0:
            return 0  # Защита от деления на 0

        # Ошибка
        error = self.setpoint - current_value

        # интеграл и производная ошибки
        self.integral += error * dt
        if self.max_integral_error is not None:
            self.integral = max(
                -self.max_integral_error,
                min(self.max_integral_error, self.integral)
            )
        derivative = (error - self.last_error) / dt

        # пид-сигнал
        output = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
        )

        # Запоминаем для следующего шага
        self.last_error = error
        self.last_time = now

        return max(-self.output_limit, min(self.output_limit, output))

# Пример использования:
# pid = PID(1.0, 0.1, 0.05, setpoint=100)
# signal = pid.update(current_temp)


class IraypleCenter(Node):
    """Нода для приезда к 0,0,0."""

    def __init__(self):
        """Инициализация."""
        super().__init__('irayple_center')
        self.max_ang_speed = 0.3
        self.max_linear_speed = 0.01  # м/с
        self.max_linear_accel = 0.05  # м/с^2
        self.dist_tolerance = 0.005  # м
        self.angle_tolerance = 2.0  # град
        self.rate = 40
        self.irayple_scale = 0.001  # мм -> метры
        self.information_scale = 0.1

        # контур положения
        self.angle_pid = PID(
            kp=50.0,
            ki=0.3,
            setpoint=0,
            output_limit=self.max_ang_speed,
            max_integral_error=10.0
        )
        self.dist_pid = PID(
            kp=5.0,
            ki=100.0,
            setpoint=0,
            output_limit=self.max_ang_speed,
            max_integral_error=10.0
        )
        # контур скорости
        self.angle_vel_pid = PID(
            kp=0.1,
            ki=5.0,
            kd=0.0,
            setpoint=0,
            output_limit=self.max_ang_speed,
            max_integral_error=10.0
        )
        self.dist_vel_pid = PID(
            kp=0.1,
            ki=5.0,
            kd=0.0,
            setpoint=0,
            output_limit=self.max_linear_speed,
            max_integral_error=10.0
        )

        # флаг получения данных с камеры
        self.get_data = False
        self._last_irayple_time = 0.0
        self._irayple_timeout_s = 0.5
        # флаг цель рассчитана один раз при первом qr
        self.calculated = False
        # флаг получены данные odom
        self.odom_received = False
        self.odom_vx = 0.0
        self.odom_omega = 0.0
        # интегратор задания w
        self._omega_sp = 0.0
        # интегратор задания v
        self._v_sp = 0.0
        self._last_timer_time = time.time()
        # флаг для старта центровки извне
        self._centering_active = False

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

        # флаг для старта центровки извне
        self._centration_flag_pub = self.create_publisher(
            Bool, '/centration_flag', 10
        )
        self.create_subscription(
            Bool, '/centration_flag', self._on_flag, 10
        )

        # rotate1 -> drive2 -> rotate3 -> drive3 -> wait_updates -> done
        self._stage = 'ready'

        self._target_angle = 0.0
        self._target_x = 0.0
        self._target_y = 0.0
        self._target_dist = 0.0
        self._rotate3_target = None  # угол для финального поворота
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

        self.declare_parameter('global_frame', 'map')

    def calculate_angle(self):
        """Угол к центру в глобальной системе координат."""
        angle_in_robot = math.atan2(self.irayple_y, self.irayple_x)
        return normalize_angle(self.odom_theta + angle_in_robot)

    def calculate_dist(self):
        """Расстояние до центра по камере (м)."""
        return math.sqrt(self.irayple_x**2 + self.irayple_y**2)

    def reset_pi(self):
        """Сброс состояния регуляторов."""
        self.angle_pid.reset()
        self.dist_pid.reset()
        self.angle_vel_pid.reset()
        self.dist_vel_pid.reset()
        self._omega_sp = 0.0
        self._v_sp = 0.0
        self._last_timer_time = time.time()

    def odom_callback(self, msg):
        """Колбек для чтения odom."""
        self.odom_received = True
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_theta = quaternion_to_yaw(msg.pose.pose.orientation)
        self.odom_vx = msg.twist.twist.linear.x
        self.odom_omega = msg.twist.twist.angular.z

    def irayple_callback(self, msg):
        """Колбек чтения irayple."""
        self.get_data = True
        self._last_irayple_time = time.time()
        self.irayple_x = msg.x * self.irayple_scale
        self.irayple_y = msg.y * self.irayple_scale
        # information: 1-3599
        self.irayple_theta = float(msg.information)
        self.irayple_byte = msg.byte
        if not self.calculated and self.odom_received:
            self.calculated = True
            d = self.calculate_dist()
            a = self.calculate_angle()
            self.get_logger().info(
                f'Центр: dist={d:.4f} м, угол={math.degrees(a):.1f}'
            )

    def _on_flag(self, msg: Bool):
        if not msg.data:
            return
        self._centering_active = msg.data

    def timer_callback(self):
        """Колбек для таймера."""
        if self._centering_active:
            now = time.time()
            dt = now - self._last_timer_time
            if dt <= 0.0:
                dt = 1.0 / self.rate
            dt = min(dt, 0.2)  # защита от больших пауз
            self._last_timer_time = now

            # если камера давно не обновлялись-стоп и ждём
            self.get_data = (
                (now - self._last_irayple_time) <= self._irayple_timeout_s
            )

            # поднимаем флаг если данные с камеры полуены
            if self.get_data:
                self.publisher_irayple_is_read.publish(String(data='True'))
            else:
                self.publisher_irayple_is_read.publish(String(data='False'))
                self.publisher_cmd_vel.publish(Twist())
                return

            # Едем к цели только если она уже рассчитана
            if not self.calculated:
                return

            if not self._centering_active:
                # гарантируем остановку
                cmd = Twist()
                self.publisher_cmd_vel.publish(cmd)
                return

            cmd = Twist()
            angle_tol_deci = self.angle_tolerance * 10.0 / 2  # град -> 0.1град
            # (почему -то надо делить на 2)

            # запускаем единый контур: считаем угол и сразу едем к центру
            if self._stage == 'ready':
                # расчет угла
                dx = self.irayple_x - self._target_x
                dy = self.irayple_y - self._target_y
                angle = math.atan2(dy, dx)
                angle = normalize_angle(angle)
                angle = math.degrees(angle)
                angle = round(angle * 10.0)  # град -> 0.1град

                # выбор направления движения по наименьшему углу
                err_front = shortest_angle_error_deci(
                    angle, self.irayple_theta
                )
                err_back = shortest_angle_error_deci(
                    angle + 1800.0, self.irayple_theta
                )
                if abs(err_back) < abs(err_front):
                    self.reverse = True
                    self._go_angle_deci = angle + 1800.0
                else:
                    self.reverse = False
                    self._go_angle_deci = angle

                # self.get_logger().info(
                #     'Запуск центровки: считаю угол и еду сразу в центр.'
                # )

                err = shortest_angle_error_deci(
                    self._go_angle_deci,
                    self.irayple_theta
                )

                if abs(err) > angle_tol_deci:
                    self.angle_pid.setpoint = 0.0
                    self.angle_pid.update(self.odom_theta)
                    print(err)
                    w = self.angle_pid.update(err)  # dw/dt
                    self._omega_sp += w * dt
                    self._omega_sp = max(
                        -self.max_ang_speed,
                        min(self.max_ang_speed, self._omega_sp)
                    )
                    self.angle_vel_pid.setpoint = self._omega_sp
                    cmd.angular.z = self.angle_vel_pid.update(self.odom_omega)
                else:
                    self.reset_pi()
                    # остановка до паузы
                    self.publisher_cmd_vel.publish(Twist())
                    time.sleep(1.0)
                    self._stage = 'go_center'
                    self.get_logger().info('rotate1 выполнен.')

            # Единый этап: угол до центра по (x,y) + движение в центр
            if self._stage == 'go_center':
                dx = self.irayple_x - self._target_x
                dy = self.irayple_y - self._target_y
                err_dist = abs(dx) + abs(dy)

                # Мягкая коррекция по углу во время движения к центру
                err_yaw_go = shortest_angle_error_deci(
                    self._go_angle_deci,
                    self.irayple_theta
                )

                if abs(err_dist) > self.dist_tolerance:

                    # проверяем реверс
                    if self.reverse:
                        dv_sp = self.dist_pid.update(-err_dist)  # dv/dt
                    else:
                        dv_sp = self.dist_pid.update(err_dist)  # dv/dt

                    self._v_sp += dv_sp * dt
                    self._v_sp = max(
                        -self.max_linear_speed,
                        min(self.max_linear_speed, self._v_sp)
                    )
                    self.dist_vel_pid.setpoint = self._v_sp
                    cmd.linear.x = self.dist_vel_pid.update(self.odom_vx)

                    # небольшой доворот PID-контуром, чтобы не уводило по курсу
                    w = self.angle_pid.update(err_yaw_go)  # dw/dt
                    self._omega_sp += w * dt
                    self._omega_sp = max(
                        -0.35 * self.max_ang_speed,
                        min(0.35 * self.max_ang_speed, 0.035 * self._omega_sp)
                    )
                    self.angle_vel_pid.setpoint = self._omega_sp
                    cmd.angular.z = self.angle_vel_pid.update(self.odom_omega)

                else:
                    self.reset_pi()
                    self.publisher_cmd_vel.publish(Twist())
                    time.sleep(1.0)
                    self._stage = 'rotate2'
                    self.get_logger().info('go_center выполнен.')

            err = shortest_angle_error_deci(
                self._target_angle,
                self.irayple_theta
            )

            if self._stage == 'rotate2':
                if abs(err) > angle_tol_deci:
                    self.angle_pid.setpoint = 0.0
                    self.angle_pid.update(self.odom_theta)
                    print(err)
                    w = self.angle_pid.update(err)  # dw/dt
                    self._omega_sp += w * dt
                    self._omega_sp = max(
                        -self.max_ang_speed,
                        min(self.max_ang_speed, self._omega_sp)
                    )
                    self.angle_vel_pid.setpoint = self._omega_sp
                    cmd.angular.z = self.angle_vel_pid.update(self.odom_omega)
                else:
                    self.reset_pi()
                    # остановка до паузы
                    self.publisher_cmd_vel.publish(Twist())
                    time.sleep(1.0)
                    self._stage = 'wait_updates'
                    self.get_logger().info('rotate1 выполнен.')

            # Ждём, пока придут свежая одометрия и данные с камеры
            if self._stage == 'wait_updates':
                cmd = Twist()  # стоп, пока ждём
                if self.odom_received and self.get_data:

                    atol = self.angle_tolerance * 10.0 / 2
                    disttolx = disttoly = self.dist_tolerance
                    if (
                        abs(self.odom_theta - self._target_angle) < atol
                        and abs(self.irayple_x) < disttolx
                        and abs(self.irayple_y) < disttoly
                    ):
                        self.get_logger().info('Центровка выполнена.')
                        self._centering_active = False
                        self._centration_flag_pub.publish(
                            Bool(data=self._centering_active)
                        )
                        self._stage = 'ready'
                    else:
                        self.get_logger().info('Центровка не выполнена.')
                        # тк тут мы не опукаем флаг центровки, то центровка
                        # начнется с этапа поворота до угла подъезда к центру
                        self._stage = 'ready'

            self.publisher_cmd_vel.publish(cmd)


def main(args=None):
    """Мейн."""
    rclpy.init(args=args)
    node = IraypleCenter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

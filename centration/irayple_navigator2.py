#!/usr/bin/env python3
"""Нода для центровки робота по QR-коду."""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
import time
import math
import argparse
from threading import Event
from irayple.geometry_utils import euler_from_quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from irayple.msg import Irayple
from nav2_msgs.action import Spin


ODOM_TOPIC = '/odom'
IRAYLE_TOPIC = 'irayple'
CENTER_CMD_VEL_TOPIC = '/cmd_vel'

IRAYPLE_CENTER_ACTION = 'irayple_center_action'

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
    # Датчик дает угол в формате 1..3599 (0.1 градуса на шаг).
    # Нормализуем во внутренний круг 0..3599,
    # чтобы корректно учитывать переброс.
    t = (target_deci - 1.0) % 3600.0
    c = (current_deci - 1.0) % 3600.0
    return (t - c + 1800.0) % 3600.0 - 1800.0


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

    def __init__(self, parser_topic=None, action_name=None):
        """Инициализация."""
        topic_for_name = parser_topic or IRAYLE_TOPIC
        safety_postfix = topic_for_name.replace('/', '_').replace('-', '_')
        super().__init__(f'irayple_center_{safety_postfix}')
        self.declare_parameter('parse_topic', IRAYLE_TOPIC)
        self.declare_parameter('action_name', IRAYPLE_CENTER_ACTION)
        self.parser_topic = (
            parser_topic
            if parser_topic is not None
            else str(self.get_parameter('parse_topic').value)
        )
        self.action_name = (
            action_name
            if action_name is not None
            else str(self.get_parameter('action_name').value)
        )
        self.max_ang_speed = 0.3
        self.max_linear_speed = 0.01  # м/с
        self.max_linear_accel = 0.05  # м/с^2
        self.dist_tolerance = 0.005  # м
        self.angle_tolerance = 2.0  # град
        self.rate = 40
        self.irayple_scale = 0.001  # мм -> метры

        self.disttol = 40
        self.angletol = 30

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
        self._action_done_event = Event()
        self._action_success = False

        self.subscription_irayple = self.create_subscription(
            Irayple,
            self.parser_topic,
            self.irayple_callback,
            10
        )

        self.subscription_odom = self.create_subscription(
            Odometry,
            ODOM_TOPIC,
            self.odom_callback,
            10
        )

        self.publisher_cmd_vel = self.create_publisher(
            Twist,
            CENTER_CMD_VEL_TOPIC,
            10
        )
        self.publisher_irayple_is_read = self.create_publisher(
            String,
            '/irayple_is_read',
            10
        )

        # сервер экшена центровки
        self._center_action_server = ActionServer(
            self,
            Spin,
            self.action_name,
            execute_callback=self._execute_center_action,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )

        # rotate1 -> drive2 -> rotate3 -> drive3 -> wait_updates -> done
        self._stage = 'ready'

        # коорды к которым ехать
        self._end_angle = 1805.0
        self._target_x = 0.0
        self._target_y = 0.0

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

    def _goal_callback(self, _goal_request):
        if self._centering_active:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _execute_center_action(self, goal_handle):
        """Выполнение экшена центровки."""
        def _make_result(started_time: float):
            """Создание результата экшена."""
            result = Spin.Result()
            elapsed = time.time() - started_time
            result.total_elapsed_time.sec = int(elapsed)
            result.total_elapsed_time.nanosec = int(
                (elapsed - int(elapsed)) * 1e9
            )
            return result

        # Используем target_yaw из Spin.Goal как входной end_angle (в 0.1 град)
        self._end_angle = math.degrees(goal_handle.request.target_yaw) * 10.0
        self._stage = 'ready'
        self._action_success = False
        self._action_done_event.clear()
        self._centering_active = True

        started = time.time()
        while rclpy.ok() and not self._action_done_event.is_set():
            if goal_handle.is_cancel_requested:
                self._centering_active = False
                self.publisher_cmd_vel.publish(Twist())
                goal_handle.canceled()
                return _make_result(started)
            time.sleep(0.05)

        result = _make_result(started)
        if self._action_success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

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

            # для последнего доворота используем _end_angle
            # рассчитаем target angle, если угол не круглый
            if self._end_angle not in [0.0, 900.0, 1800.0, 2700.0, 3600.0]:
                err1 = shortest_angle_error_deci(0.0, self._end_angle)
                err2 = shortest_angle_error_deci(900.0, self._end_angle)
                err3 = shortest_angle_error_deci(1800.0, self._end_angle)
                err4 = shortest_angle_error_deci(2700.0, self._end_angle)
                err5 = shortest_angle_error_deci(3600.0, self._end_angle)
                m = min(abs(err1), abs(err2), abs(err3), abs(err4), abs(err5))
                if m == abs(err1) or m == abs(err5):
                    self._target_angle = 0.0
                elif m == abs(err2):
                    self._target_angle = 900.0
                elif m == abs(err3):
                    self._target_angle = 1800.0
                elif m == abs(err4):
                    self._target_angle = 2700.0
            else:
                self._target_angle = self._end_angle

            # запускаем
            if self._stage == 'ready' and self._centering_active:
                # self._target_angle = self.calculate_angle()
                self._stage = 'rotate1'
                self.get_logger().info('Запуск центровки.')

            cmd = Twist()
            angle_tol_deci = self.angle_tolerance * 10.0 / 2  # град -> 0.1град
            # (почему -то надо делить на 2)

            # 1 угол правильный
            if self._stage == 'rotate1':
                err = shortest_angle_error_deci(
                    self._target_angle, self.irayple_theta
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
                    self._stage = 'drive2'
                    self.get_logger().info('rotate1 выполнен.')

            # 2 Проезд к x = 0
            if self._stage == 'drive2':
                if (
                    self._target_angle in [900.0, 2700.0, 0.0, 1800.0, 3600.0]
                ):
                    if (self._target_angle == 1800.0):
                        # вперед/назад ехать
                        koef = -1.0
                        err_dist = (
                            self.irayple_y + self._target_y - koef * 0.350
                        )
                        if self.irayple_x < 0:
                            self.koef_angle = 1.0
                        else:
                            self.koef_angle = -1.0
                    elif (
                        (self._target_angle == 0.0) or
                        (self._target_angle == 3600.0)
                    ):
                        koef = 1.0
                        err_dist = (
                            self.irayple_y + self._target_y - koef * 0.350
                        )
                        if self.irayple_x < 0:
                            self.koef_angle = -1.0
                        else:
                            self.koef_angle = 1.0
                    elif (self._target_angle == 2700.0):
                        koef = -1.0
                        err_dist = (
                            self.irayple_x + self._target_x - koef * 0.350
                        )
                        if self.irayple_y < 0:
                            self.koef_angle = -1.0
                        else:
                            self.koef_angle = 1.0
                    elif (self._target_angle == 900.0):
                        koef = 1.0
                        err_dist = (
                            self.irayple_x + self._target_x - koef * 0.350
                        )
                        if self.irayple_y < 0:
                            self.koef_angle = 1.0
                        else:
                            self.koef_angle = -1.0
                print(err_dist)
                if abs(err_dist) > self.dist_tolerance:
                    dv_sp = self.dist_pid.update(err_dist)  # dv/dt
                    self._v_sp += dv_sp * dt
                    self._v_sp = max(
                        -self.max_linear_speed,
                        min(self.max_linear_speed, self._v_sp)
                    )
                    self.dist_vel_pid.setpoint = self._v_sp
                    cmd.linear.x = koef * self.dist_vel_pid.update(
                        self.odom_vx
                    )
                else:
                    self.reset_pi()
                    # остановка до паузы
                    self.publisher_cmd_vel.publish(Twist())
                    time.sleep(1.0)
                    self._stage = 'rotate3'
                    self.get_logger().info('drive2 выполнен.')

            # 3 центруем ось перпендикулярную (поворот)
            if self._stage == 'rotate3':
                err = shortest_angle_error_deci(
                    self._target_angle + self.koef_angle * 330.0,
                    self.irayple_theta
                )
                print(err)
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
                    self._stage = 'drive4'
                    self.get_logger().info('rotate3 выполнен.')

            # 4 центруем ось перпендикулярную (проезд)
            if self._stage == 'drive4':
                if (
                    self._target_angle in [900.0, 2700.0, 0.0, 1800.0, 3600.0]
                ):
                    if (self._target_angle == 1800.0):
                        err_dist = self.irayple_x + self._target_x
                        # вперед/назад ехать

                    elif (
                        (self._target_angle == 0.0) or
                        (self._target_angle == 3600.0)
                    ):
                        err_dist = self.irayple_x + self._target_x
                    elif (self._target_angle == 2700.0):
                        err_dist = self.irayple_y + self._target_y
                    elif (self._target_angle == 900.0):
                        err_dist = self.irayple_y + self._target_y
                    print(err_dist)
                # наиболее действенный метод
                # else:
                #     err_dist = self.irayple_x
                #     koef = 1.0
                print(err_dist)
                if abs(err_dist) > self.dist_tolerance:
                    dv_sp = self.dist_pid.update(err_dist)  # dv/dt
                    self._v_sp += dv_sp * dt
                    self._v_sp = max(
                        -self.max_linear_speed,
                        min(self.max_linear_speed, self._v_sp)
                    )
                    self.dist_vel_pid.setpoint = self._v_sp
                    cmd.linear.x = - abs(self.dist_vel_pid.update(
                        self.odom_vx
                    ))
                else:
                    self.reset_pi()
                    # остановка до паузы
                    self.publisher_cmd_vel.publish(Twist())
                    time.sleep(1.0)
                    self._stage = 'rotate5'
                    self.get_logger().info('drive4 выполнен.')

            # 5 центруем ось перпендикулярную (поворот)
            if self._stage == 'rotate5':
                err = shortest_angle_error_deci(
                    self._target_angle, self.irayple_theta
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
                    self._stage = 'drive6'
                    self.get_logger().info('rotate5 выполнен.')

            # 6 центруем ось продольную (проезд)
            if self._stage == 'drive6':
                if (
                    self._target_angle in [900.0, 2700.0, 0.0, 1800.0, 3600.0]
                ):
                    if self._target_angle == 900.0:
                        err_dist = self.irayple_x + self._target_x
                        koef = 1.0
                    elif (self._target_angle == 2700.0):
                        err_dist = self.irayple_x + self._target_x
                        koef = -1.0
                    elif (
                        (self._target_angle == 0.0) or
                        (self._target_angle == 3600.0)
                    ):
                        err_dist = self.irayple_y + self._target_y
                        koef = 1.0
                    elif (self._target_angle == 1800.0):
                        err_dist = self.irayple_y + self._target_y
                        koef = -1.0
                # else:
                #     err_dist = self.irayple_y
                #     koef = 1.0
                print(err_dist)
                if abs(err_dist) > self.dist_tolerance:
                    dv_sp = self.dist_pid.update(err_dist)  # dv/dt
                    self._v_sp += dv_sp * dt
                    self._v_sp = max(
                        -self.max_linear_speed,
                        min(self.max_linear_speed, self._v_sp)
                    )
                    self.dist_vel_pid.setpoint = self._v_sp
                    cmd.linear.x = koef * self.dist_vel_pid.update(
                        self.odom_vx
                    )
                else:
                    self.reset_pi()
                    # остановка до паузы
                    self.publisher_cmd_vel.publish(Twist())
                    time.sleep(1.0)
                    self._stage = 'rotate7'
                    self.get_logger().info('drive6 выполнен.')

            # 7 финальный поворот по углу
            if self._stage == 'rotate7':
                err = shortest_angle_error_deci(
                    self._end_angle, self.irayple_theta
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
                    self._stage = 'wait_updates'
                    self.get_logger().info('rotate7 выполнен.')

            # Ждём, пока придут свежая одометрия и данные с камеры
            if self._stage == 'wait_updates':
                cmd = Twist()  # стоп, пока ждём
                if self.odom_received and self.get_data:

                    if (
                        abs(abs(shortest_angle_error_deci(
                            self._end_angle, self.irayple_theta
                        ))) < self.angletol
                        and abs(self.irayple_x) < self.disttol
                        and abs(self.irayple_y) < self.disttol
                    ):
                        self.get_logger().info('Центровка выполнена.')
                        self._action_success = True
                        self._centering_active = False
                        self._action_done_event.set()
                        self._stage = 'ready'
                    else:
                        self.get_logger().info('Центровка не выполнена.')
                        # тк тут мы не опукаем флаг центровки, то центровка
                        # начнется с этапа поворота до угла подъезда к центру

                        # для отладки, после закоментить
                        self._action_success = False
                        self._centering_active = False
                        self._action_done_event.set()
                        self._stage = 'ready'

            self.publisher_cmd_vel.publish(cmd)


def main(args=None):
    """Мейн."""
    import sys

    rclpy.init(args=args)
    kwargs = {}
    if '--parse_topic' in sys.argv or '--action_name' in sys.argv:
        ap = argparse.ArgumentParser()
        ap.add_argument('--parse_topic', default=IRAYLE_TOPIC)
        ap.add_argument('--action_name', default=IRAYPLE_CENTER_ACTION)
        cli, _ = ap.parse_known_args()
        kwargs = {
            'parser_topic': cli.parse_topic,
            'action_name': cli.action_name,
        }
    node = IraypleCenter(**kwargs)
    # запускаем второй поток, чтобы не блокировать основной
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

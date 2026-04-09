#!/usr/bin/env python3
import math
import threading

import rclpy
from rclpy.action import (
    ActionClient,
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.time import Time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import FollowPath, NavigateToPose, Spin
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion

from irayple.msg import Irayple

TARGET_ANGLE = 270.0
TARGET_ANGLE_RAD = math.radians(TARGET_ANGLE)
PRODOLNI_OTEZD = 0.3  # м
GOAL_STATUS_TEXT = {
    GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
    GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
    GoalStatus.STATUS_EXECUTING: 'EXECUTING',
    GoalStatus.STATUS_CANCELING: 'CANCELING',
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_to_quaternion(yaw_rad: float):
    """кватернион из угла yaw."""
    q = Quaternion()
    half = 0.5 * float(yaw_rad)
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


class SimpleCenteringNavigator(Node):
    """Класс для центровки робота по QR."""

    def __init__(self):
        """Инициализация."""
        super().__init__('irayple_navigator')

        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('target_yaw_deg', TARGET_ANGLE)
        self.declare_parameter('min_goal_distance_m', 0.003)
        self.declare_parameter('min_goal_yaw_deg', 4.0)
        self.declare_parameter('yaw_settle_band_deg', 4.0)
        self.declare_parameter('epselon_yaw_deg', 30.0)
        self.declare_parameter('epselon_lateral_m', 0.02)
        self.declare_parameter('progress_required_movement_radius', 0.05)
        self.declare_parameter('progress_movement_time_allowance', 20.0)
        self.declare_parameter('centering_action_name', '/centration_navigate_to_pose')
        self.declare_parameter('behavior_tree_xml', '/home/ubuntu/ros2_ws/src/navigation/behavior_trees/navigate_to_pose_precise.xml')

        self._centering_active = False
        self._active_lock = threading.Lock()
        self._last_irayple_xy_m = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._controller_params_client = AsyncParameterClient(self, '/controller_server')
        self._nav_client = ActionClient(
            self, NavigateToPose, '/mppi_navigate_to_pose'
        )
        self._spin_client = ActionClient(self, Spin, '/spin')
        self._centering_server = ActionServer(
            self,
            NavigateToPose,
            str(self.get_parameter("centering_action_name").value),
            execute_callback=self._execute_centering_action,
            goal_callback=self._on_centering_goal,
            cancel_callback=self._on_centering_cancel,
        )
        self._centration_flag_pub = self.create_publisher(
            Bool, '/centration_flag', 10
        )
        self.create_subscription(
            Bool, '/centration_flag', self._on_flag, 10
        )
        self.create_subscription(
            Irayple, '/irayple', self._on_irayple, 10
        )

        self.get_logger().info('Centering Navigator started')

    def _on_irayple(self, msg: Irayple):
        self._last_irayple_xy_m = (msg.x / 10000.0, msg.y / 10000.0)

    def _on_flag(self, msg: Bool):
        if not msg.data:
            return
        with self._active_lock:
            if self._centering_active:
                self.get_logger().warn(
                    'Centering already active, ignore repeated True flag'
                )
                return
            self._centering_active = True
        self.get_logger().info(
            'Centration flag received, starting centering...'
        )
        threading.Thread(target=self._run_from_flag, daemon=True).start()

    def _publish_flag(self, value: bool):
        self._centration_flag_pub.publish(Bool(data=bool(value)))

    def _status_to_text(self, status: int) -> str:
        return GOAL_STATUS_TEXT.get(status, f'UNKNOWN({status})')

    def _apply_progress_checker(self):
        required_radius = float(
            self.get_parameter('progress_required_movement_radius').value
        )
        allowance = float(
            self.get_parameter('progress_movement_time_allowance').value
        )
        if not self._controller_params_client.wait_for_services(timeout_sec=1.0):
            self.get_logger().warn(
                'controller_server params service unavailable, skip progress checker set'
            )
            return
        params = [
            Parameter(
                'progress_checker.required_movement_radius',
                Parameter.Type.DOUBLE,
                required_radius,
            ),
            Parameter(
                'progress_checker.movement_time_allowance',
                Parameter.Type.DOUBLE,
                allowance,
            ),
        ]
        future = self._controller_params_client.set_parameters(params)
        done_event = threading.Event()
        result_holder = {'ok': False}

        def _on_set_done(done_future):
            try:
                response = done_future.result()
                results = getattr(response, 'results', None)
                if results is None:
                    result_holder['ok'] = bool(getattr(response, 'successful', False))
                else:
                    result_holder['ok'] = all(r.successful for r in results)
            except Exception as exc:
                self.get_logger().warn(
                    f'Failed to set progress checker params: {exc}'
                )
            done_event.set()

        future.add_done_callback(_on_set_done)
        done_event.wait(timeout=2.0)
        self.get_logger().info(
            f'Progress checker: radius={required_radius:.3f}, allowance={allowance:.1f}s, applied={result_holder["ok"]}'
        )

    def _get_robot_pose(self):
        global_frame = str(self.get_parameter('global_frame').value)
        base_frame = str(self.get_parameter('base_frame').value)
        timeout = Duration(seconds=1.5)
        if not self._tf_buffer.can_transform(
            global_frame, base_frame, Time(), timeout
        ):
            self.get_logger().error(
                f'TF unavailable: {global_frame}->{base_frame}'
            )
            return None
        tf = self._tf_buffer.lookup_transform(global_frame, base_frame, Time())
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        _, _, yaw = euler_from_quaternion(
            [tf.transform.rotation.x,
             tf.transform.rotation.y,
             tf.transform.rotation.z,
             tf.transform.rotation.w]
        )
        return x, y, yaw

    def pose_of_qr(self):
        """получение позиции 0 0 yaw в глобальной системе координат."""
        if self._last_irayple_xy_m is None:
            self.get_logger().error(
                'No /irayple data, cannot center from flag mode'
            )
            return None
        pose = self._get_robot_pose()
        if pose is None:
            return None
        robot_x, robot_y, robot_yaw = pose
        qr_x_cam, qr_y_cam = self._last_irayple_xy_m
        target_yaw = math.radians(float(self.get_parameter(
            'target_yaw_deg').value)
        )
        cos_y = math.cos(robot_yaw)
        sin_y = math.sin(robot_yaw)
        target_x = robot_x + (qr_x_cam * cos_y - qr_y_cam * sin_y)
        target_y = robot_y + (qr_x_cam * sin_y + qr_y_cam * cos_y)
        return target_x, target_y, target_yaw

    def _build_nav_goal(self, x: float, y: float, yaw: float):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = str(self.get_parameter(
            'global_frame').value
        )
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation = yaw_to_quaternion(yaw)
        goal.behavior_tree = str(self.get_parameter(
            'behavior_tree_xml').value
        )
        return goal

    def _send_nav_goal_and_wait(self, x: float, y: float, yaw: float):
        """Отправляет один NavigateToPose goal и ждет результат."""
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server unavailable')
            return GoalStatus.STATUS_ABORTED

        done_event = threading.Event()
        status_holder = {'status': GoalStatus.STATUS_UNKNOWN}

        def _on_result(result_future):
            try:
                status_holder['status'] = result_future.result().status
            except Exception as exc:
                self.get_logger().error(f'Result callback failed: {exc}')
                status_holder['status'] = GoalStatus.STATUS_ABORTED
            done_event.set()

        def _on_goal_response(send_future):
            try:
                gh = send_future.result()
            except Exception as exc:
                self.get_logger().error(f'Send goal failed: {exc}')
                status_holder['status'] = GoalStatus.STATUS_ABORTED
                done_event.set()
                return
            if not gh.accepted:
                self.get_logger().error('Goal rejected by graceful planer')
                status_holder['status'] = GoalStatus.STATUS_ABORTED
                done_event.set()
                return
            gh.get_result_async().add_done_callback(_on_result)

        self.get_logger().info(
            f'Send goal to qr: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}'
        )
        self._nav_client.send_goal_async(
            self._build_nav_goal(x, y, yaw)
        ).add_done_callback(_on_goal_response)

        done_event.wait()
        if status_holder['status'] == GoalStatus.STATUS_ABORTED:
            pose = self._get_robot_pose()
            if pose is not None:
                rx, ry, ryaw = pose
                distance_error = math.hypot(x - rx, y - ry)
                yaw_error = abs(normalize_angle(ryaw - yaw))
                min_dist = float(self.get_parameter('min_goal_distance_m').value)
                min_yaw = math.radians(
                    float(self.get_parameter('min_goal_yaw_deg').value)
                )
                if distance_error <= min_dist:
                    self.get_logger().warn(
                        'Goal aborted, but robot is already within distance tolerance'
                    )
                    return GoalStatus.STATUS_SUCCEEDED
        return status_holder['status']

    def _send_spin_and_wait(self, yaw_delta: float) -> int:
        """Rotate in place with Nav2 Spin action."""
        if not self._spin_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Spin action server unavailable')
            return GoalStatus.STATUS_ABORTED

        goal = Spin.Goal()
        goal.target_yaw = float(yaw_delta)
        goal.time_allowance = Duration(seconds=20.0).to_msg()

        done_event = threading.Event()
        status_holder = {'status': GoalStatus.STATUS_UNKNOWN}

        def _on_result(result_future):
            try:
                status_holder['status'] = result_future.result().status
            except Exception as exc:
                self.get_logger().error(f'Spin result callback failed: {exc}')
                status_holder['status'] = GoalStatus.STATUS_ABORTED
            done_event.set()

        def _on_goal_response(send_future):
            try:
                gh = send_future.result()
            except Exception as exc:
                self.get_logger().error(f'Spin send goal failed: {exc}')
                status_holder['status'] = GoalStatus.STATUS_ABORTED
                done_event.set()
                return
            if not gh.accepted:
                self.get_logger().error('Spin goal rejected')
                status_holder['status'] = GoalStatus.STATUS_ABORTED
                done_event.set()
                return
            gh.get_result_async().add_done_callback(_on_result)

        self.get_logger().info(f'Send spin: delta_yaw={yaw_delta:.3f}')
        self._spin_client.send_goal_async(goal).add_done_callback(_on_goal_response)
        done_event.wait()
        return status_holder['status']

    def _run_from_flag(self):
        # Обновляем параметры progress checker перед сессией.
        self._apply_progress_checker()
        # epselon = math.radians(self.get_parameter('epselon_yaw_deg').value)
        minimum_distance = self.get_parameter('min_goal_distance_m').value
        # Порог по углу (рад).
        min_yaw_error = math.radians(
            self.get_parameter('min_goal_yaw_deg').value
        )
        while self._centering_active:
            # Читаем цель из QR на каждой итерации: после движения оценка может
            # заметно измениться, и фиксировать цель на всю сессию нельзя.
            qr_pose = self.pose_of_qr()
            if qr_pose is None:
                self.get_logger().error('No qr pose, cannot center')
                self._finish(False)
                return False
            qr_x, qr_y, qr_yaw = qr_pose

            robot_pose = self._get_robot_pose()
            if robot_pose is None:
                self.get_logger().error('No robot pose, cannot center')
                self._finish(False)
                return False

            # Текущая поза робота в map.
            rx, ry, ryaw = robot_pose

            # Целевой угол в рад.
            self.target_angle = TARGET_ANGLE_RAD
            if abs(qr_yaw - TARGET_ANGLE_RAD) > math.pi:
                self.target_angle = TARGET_ANGLE_RAD + math.pi
            self.get_logger().info(f'target_angle: {self.target_angle}')

            # Если уже в целевых допусках — завершаем.
            # вектор от робота к цели.
            dx = qr_x - rx
            dy = qr_y - ry
            distance = math.hypot(dx, dy)
            # ошибка по углу.
            yaw_error = abs(normalize_angle(ryaw - self.target_angle))
            if distance <= minimum_distance and yaw_error <= min_yaw_error:
                self._finish(True)
                return True
            if distance <= minimum_distance and yaw_error > min_yaw_error:
                spin_status = self._send_spin_and_wait(
                    normalize_angle(self.target_angle - ryaw)
                )
                self.get_logger().info(
                    f'spin status: {self._status_to_text(spin_status)}'
                )
                if spin_status != GoalStatus.STATUS_SUCCEEDED:
                    self.get_logger().error('Failed final spin alignment')
                    self._finish(False)
                    return False
                continue

            # Ошибки в текущей локальной системе робота (ось X вперед, Y влево).
            # Для back/forward-логики важна именно поперечная ошибка относительно робота.
            delta_longitudinal = dx * math.cos(ryaw) + dy * math.sin(ryaw)
            delta_lateral = -dx * math.sin(ryaw) + dy * math.cos(ryaw)
            # если delta_longitudinal > 0, то цель впереди
            # если delta_longitudinal < 0, то цель позади
            # если delta_lateral > 0, то цель справа
            # если delta_lateral < 0, то цель слева

            # Если боковая ошибка малая — едем сразу в цель,
            # иначе делаем back -> forward.
            if abs(delta_lateral) < self.get_parameter('epselon_lateral_m').value:
                # Пока не доехали по XY, не заставляем держать target yaw:
                # это убирает дрожание около 270-273 градусов.
                direct_goal_yaw = ryaw if distance > minimum_distance else self.target_angle
                if abs(normalize_angle(ryaw - self.target_angle)) <= min_yaw_error:
                    direct_goal_yaw = ryaw
                status = self._send_nav_goal_and_wait(qr_x, qr_y, direct_goal_yaw)
                self.get_logger().info(f'status: {self._status_to_text(status)}')
                if status != GoalStatus.STATUS_SUCCEEDED:
                    self.get_logger().error('Failed to navigate to qr')
                    self._finish(False)
                    return False
            else:
                angle = math.atan2(delta_lateral, PRODOLNI_OTEZD)
                if delta_lateral > 0:
                    back_angle = self.target_angle - angle
                    back_lateral = delta_lateral
                    back_longitudinal = PRODOLNI_OTEZD
                else:
                    back_angle = self.target_angle + angle
                    back_lateral = -delta_lateral
                    back_longitudinal = PRODOLNI_OTEZD

                # Едем назад.
                # Переводим продольное/поперечное смещение из локальной оси
                # target_angle в глобальные координаты map.
                back_x = (
                    rx
                    + back_longitudinal * math.cos(back_angle)
                    - back_lateral * math.sin(back_angle)
                )
                back_y = (
                    ry
                    + back_longitudinal * math.sin(back_angle)
                    + back_lateral * math.cos(back_angle)
                )
                status = self._send_nav_goal_and_wait(
                    back_x,
                    back_y,
                    back_angle
                )
                self.get_logger().info(f'status: {self._status_to_text(status)}')
                if status != GoalStatus.STATUS_SUCCEEDED:
                    self.get_logger().error('Failed go back')
                    self._finish(False)
                    return False

                # Едем к цели.
                robot_pose = self._get_robot_pose()
                if robot_pose is None:
                    self._finish(False)
                    return False
                _, _, ryaw = robot_pose
                status = self._send_nav_goal_and_wait(
                    qr_x, qr_y, ryaw
                )
                self.get_logger().info(f'status: {self._status_to_text(status)}')
                if status != GoalStatus.STATUS_SUCCEEDED:
                    self.get_logger().error('Failed to navigate to qr')
                    self._finish(False)
                    return False

            # проверяем достижение по свежей позе.
            # перечитываем TF после движения.
            robot_pose = self._get_robot_pose()
            if robot_pose is None:
                self._finish(False)
                return False
            rx, ry, ryaw = robot_pose
            if (
                math.hypot(qr_x - rx, qr_y - ry) <= minimum_distance and
                abs(normalize_angle(ryaw - self.target_angle)) <= min_yaw_error
            ):
                self._finish(True)
                return True

            self.get_logger().warn('Target not reached yet, repeat centering loop')

    def _on_centering_goal(self, _goal_request):
        if self._centering_active:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_centering_cancel(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _execute_centering_action(self, goal_handle):
        self._centering_active = True
        success = self._run_from_flag()
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return NavigateToPose.Result()

    def _finish(self, success: bool):
        with self._active_lock:
            self._centering_active = False
        self._publish_flag(False)
        self.get_logger().info(f'Centration finished, success={success}')


def main(args=None):
    """Мейн."""
    rclpy.init(args=args)
    node = SimpleCenteringNavigator()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

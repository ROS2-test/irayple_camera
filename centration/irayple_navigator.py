#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Bool
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener, TransformException
from tf_transformations import euler_from_quaternion


def normalize_angle(a: float) -> float:
    """Normalize angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def clamp_angle_to_band(value: float, center: float, half_width: float) -> float:
    """Clamp angle to circular band [center-half_width, center+half_width]."""
    err = normalize_angle(value - center)
    if err > half_width:
        err = half_width
    elif err < -half_width:
        err = -half_width
    return normalize_angle(center + err)


def quaternion_to_yaw(q) -> float:
    """Extract yaw from geometry_msgs/Quaternion."""
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    return float(yaw)


def yaw_to_quaternion(yaw: float):
    """Create quaternion (x, y, z, w) from yaw."""
    half = 0.5 * float(yaw)
    qx = 0.0
    qy = 0.0
    qz = math.sin(half)
    qw = math.cos(half)
    return qx, qy, qz, qw


def goal_status_to_text(status: int) -> str:
    mapping = {
        GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
        GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
        GoalStatus.STATUS_EXECUTING: 'EXECUTING',
        GoalStatus.STATUS_CANCELING: 'CANCELING',
        GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
        GoalStatus.STATUS_CANCELED: 'CANCELED',
        GoalStatus.STATUS_ABORTED: 'ABORTED',
    }
    return mapping.get(int(status), f'UNMAPPED({status})')


class IraypleNavigator(Node):
    def __init__(self):
        super().__init__('irayple_navigator_graceful_centering')

        # -------- Params ----------
        self.declare_parameter('target_yaw_deg', 0.0)
        self.declare_parameter('epselon_yaw_deg', 60.0)
        self.declare_parameter('epselon_dist_m', 0.50)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('base_frame_id', 'base_footprint')

        # tolerances
        self.pos_tol_m = 0.005          # 0.5 cm
        self.yaw_tol_rad = math.radians(3.0)

        # lateral trigger: сколько отклоняться “в сторону” (перпендикуляр к роботу)
        # (ставь чуть больше pos_tol, чтобы не ловить дрожание)
        self.lateral_trigger_m = 0.006
        # trigger for forward/backward correction along robot longitudinal axis
        self.longitudinal_trigger_m = 0.006

        # retreat distance mapping: retreat ~= gain * |lateral_err| + margin
        self.declare_parameter('retreat_gain', 1.2)
        self.declare_parameter('retreat_margin_m', 0.004)
        self.declare_parameter('retreat_min_m', 0.005)
        self.declare_parameter('retreat_max_m', 0.10)

        self.retreat_gain = float(self.get_parameter('retreat_gain').value)
        self.retreat_margin_m = float(self.get_parameter('retreat_margin_m').value)
        self.retreat_min_m = float(self.get_parameter('retreat_min_m').value)
        self.retreat_max_m = float(self.get_parameter('retreat_max_m').value)

        self.yaw = math.radians(float(self.get_parameter('target_yaw_deg').value))
        self.epsilon_yaw_rad = math.radians(float(self.get_parameter('epselon_yaw_deg').value))
        self.epsilon_dist_m = float(self.get_parameter('epselon_dist_m').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.base_frame_id = str(self.get_parameter('base_frame_id').value)

        self.max_cycles = 50
        self.goal_timeout_sec = 120.0
        self.qr_timeout_sec = 0.7  # если камера/qr давно молчит — ждём

        # -------- State ----------
        self._qr_pose = None
        self._qr_time = None

        self._target_pose = None
        self._target_x = None
        self._target_y = None
        self._have_target = False
        self._centering_requested = False
        self._centering_in_progress = False
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # -------- подписки ----------
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/qr_pose',
            self._on_qr_pose,
            10
        )

        # флаг центрации
        self.create_subscription(
            Bool,
            '/centration_flag',
            self._on_centration_flag,
            10
        )
        self._centration_flag_pub = self.create_publisher(Bool, '/centration_flag', 10)

        # где брать target извне (как ты хотел — “не всегда к 0”)
        self.create_subscription(
            PoseStamped,
            '/motion_target',
            self._on_target_pose,
            10
        )

        # -------- клиент action ----------
        self.action_name = '/graceful_navigate_to_pose'
        self._client = ActionClient(self, NavigateToPose, self.action_name)

        # Behavior tree (precise)
        # Можно оставить пустым, но я зафиксировал, чтобы именно precise-XML работал.
        self.bt_xml = '/home/ubuntu/ros2_ws/src/navigation/behavior_trees/navigate_to_pose_precise.xml'

    def _on_qr_pose(self, msg: PoseWithCovarianceStamped):
        self._qr_pose = msg
        self._qr_time = self.get_clock().now()

    def _on_target_pose(self, msg: PoseStamped):
        self._target_pose = msg
        self._target_x = float(msg.pose.position.x)
        self._target_y = float(msg.pose.position.y)
        self._have_target = True

    def _on_centration_flag(self, msg: Bool):
        if msg.data:
            if not self._centering_requested and not self._centering_in_progress:
                self.get_logger().info('Received /centration_flag=True, starting centering session.')
            self._centering_requested = True

    def _publish_centration_flag(self, value: bool):
        msg = Bool()
        msg.data = bool(value)
        self._centration_flag_pub.publish(msg)

    def _yaw_in_allowed_band(self, yaw_value: float) -> bool:
        yaw_err = abs(normalize_angle(yaw_value - self.yaw))
        return yaw_err <= self.epsilon_yaw_rad

    def _recover_yaw(self, cur_x: float, cur_y: float, cur_yaw: float):
        yaw_err_deg = math.degrees(abs(normalize_angle(self.yaw - cur_yaw)))
        self.get_logger().warn(
            f'Yaw out of allowed band ({yaw_err_deg:.2f}deg). Recovering to target yaw '
            f'{math.degrees(self.yaw):.1f}deg at current position.'
        )
        # Recovery uses the same global planner, no direct velocity commands.
        self._send_graceful_goal_and_wait(cur_x, cur_y, self.yaw)

    def _compute_allowed_approach_goal(self, cur_x: float, cur_y: float, cur_yaw: float):
        """
        Compute global intermediate goal that is as close to (target_x,target_y) as possible
        while keeping heading inside [target_yaw - eps, target_yaw + eps].
        """
        travel_yaw = clamp_angle_to_band(cur_yaw, self.yaw, self.epsilon_yaw_rad)
        vx = math.cos(travel_yaw)
        vy = math.sin(travel_yaw)

        tx = self._target_x
        ty = self._target_y

        # Orthogonal projection of target onto line through current pose with heading travel_yaw.
        s = (tx - cur_x) * vx + (ty - cur_y) * vy
        gx = cur_x + s * vx
        gy = cur_y + s * vy
        return gx, gy, travel_yaw

    def _get_robot_pose_from_tf(self):
        try:
            tf_msg = self._tf_buffer.lookup_transform(
                self.frame_id, self.base_frame_id, Time()
            )
        except TransformException as ex:
            self.get_logger().warn(
                f'No TF {self.frame_id}->{self.base_frame_id}: {ex}',
                throttle_duration_sec=2.0
            )
            return None

        t = tf_msg.transform.translation
        r = tf_msg.transform.rotation
        yaw = quaternion_to_yaw(r)
        return float(t.x), float(t.y), float(yaw)

    def _within_tolerance(self, cur_x, cur_y, cur_yaw) -> bool:
        dx = self._target_x - cur_x
        dy = self._target_y - cur_y
        pos_err = math.hypot(dx, dy)
        yaw_err = abs(normalize_angle(self.yaw - cur_yaw))
        return (pos_err <= self.pos_tol_m) and (yaw_err <= self.yaw_tol_rad)

    def _compute_longitudinal_lateral_errors(self, cur_yaw: float, dx: float, dy: float):
        # Transform target vector from map frame to robot frame:
        # x_local -> longitudinal (forward), y_local -> lateral (left).
        longitudinal = dx * math.cos(cur_yaw) + dy * math.sin(cur_yaw)
        lateral = dx * (-math.sin(cur_yaw)) + dy * (math.cos(cur_yaw))
        return longitudinal, lateral

    def _compute_retreat_point(self, cur_x, cur_y, cur_yaw, longitudinal_err: float):
        # Retreat is along robot backward direction, magnitude from longitudinal error.
        d = abs(longitudinal_err) * self.retreat_gain + self.retreat_margin_m
        d = max(self.retreat_min_m, min(self.retreat_max_m, d))

        back_x = cur_x - d * math.cos(cur_yaw)
        back_y = cur_y - d * math.sin(cur_yaw)
        back_yaw = cur_yaw  # обычно лучше не пытаться поворачивать на retreat
        return back_x, back_y, back_yaw, d

    def _compute_forward_point(self, cur_x, cur_y, cur_yaw, dist_m: float):
        fwd_x = cur_x + dist_m * math.cos(cur_yaw)
        fwd_y = cur_y + dist_m * math.sin(cur_yaw)
        return fwd_x, fwd_y, cur_yaw

    def _limit_goal_distance(self, x: float, y: float):
        pose = self._get_robot_pose_from_tf()
        if pose is None:
            return x, y, False, None
        cur_x, cur_y, _ = pose
        dx = x - cur_x
        dy = y - cur_y
        dist = math.hypot(dx, dy)
        if dist <= self.epsilon_dist_m or dist < 1e-6:
            return x, y, False, dist
        scale = self.epsilon_dist_m / dist
        limited_x = cur_x + dx * scale
        limited_y = cur_y + dy * scale
        return limited_x, limited_y, True, dist

    def _send_graceful_goal_and_wait(self, x: float, y: float, yaw: float):
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f'Action server not available: {self.action_name}')
            return None, None

        goal_x, goal_y, limited, raw_dist = self._limit_goal_distance(x, y)
        if limited:
            self.get_logger().info(
                f'Goal distance limited to epselon_dist_m={self.epsilon_dist_m:.2f}m: '
                f'raw_dist={raw_dist:.3f}m, limited_goal=({goal_x:.3f},{goal_y:.3f})'
            )
        else:
            goal_x, goal_y = x, y

        qx, qy, qz, qw = yaw_to_quaternion(yaw)

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(goal_x)
        goal.pose.pose.position.y = float(goal_y)
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.x = float(qx)
        goal.pose.pose.orientation.y = float(qy)
        goal.pose.pose.orientation.z = float(qz)
        goal.pose.pose.orientation.w = float(qw)

        # фиксируем BT для precise
        goal.behavior_tree = self.bt_xml

        self.get_logger().info(
            f'Sending graceful goal: x={goal_x:.3f}, y={goal_y:.3f}, yaw={math.degrees(yaw):.1f}deg'
        )

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=self.goal_timeout_sec)
        goal_handle = send_future.result()
        if goal_handle is None:
            self.get_logger().error('send_goal_async returned None')
            return None, None
        if not goal_handle.accepted:
            self.get_logger().error(
                f'Goal REJECTED by {self.action_name}: '
                f'x={goal_x:.3f}, y={goal_y:.3f}, yaw={math.degrees(yaw):.1f}deg'
            )
            return GoalStatus.STATUS_UNKNOWN, None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.goal_timeout_sec)
        result_wrapped = result_future.result()
        if result_wrapped is None:
            self.get_logger().error('get_result_async returned None / timeout')
            return goal_handle.status, None

        status = goal_handle.status
        result = result_wrapped.result
        err_code = getattr(result, 'error_code', None)
        err_msg = getattr(result, 'error_msg', None)
        self.get_logger().info(
            f'Goal done: status={goal_status_to_text(status)}({status}), '
            f'error_code={err_code}, error_msg={err_msg}'
        )
        return status, result

    def _wait_for_robot_pose(self, timeout_sec: float = 2.0) -> bool:
        t_start = self.get_clock().now()
        while rclpy.ok():
            if self._get_robot_pose_from_tf() is not None:
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
            dt = (self.get_clock().now() - t_start).nanoseconds * 1e-9
            if dt > timeout_sec:
                return False
        return False

    def _run_centering_session(self):
        self._centering_in_progress = True
        self._centering_requested = False

        # ТЗ: центруемся строго к (0, 0) и target_yaw_deg.
        self._target_x = 0.0
        self._target_y = 0.0
        self._have_target = True

        if not self._wait_for_robot_pose(timeout_sec=3.0):
            self.get_logger().warn(
                f'Centering aborted: no TF {self.frame_id}->{self.base_frame_id}.'
            )
            self._publish_centration_flag(False)
            self._centering_in_progress = False
            return

        self.get_logger().info(
            f'Centering start -> target=(0.000,0.000), target_yaw={math.degrees(self.yaw):.1f}deg, '
            f'allowed_band=+/-{math.degrees(self.epsilon_yaw_rad):.1f}deg'
        )

        pose = self._get_robot_pose_from_tf()
        if pose is None:
            self.get_logger().warn('Centering aborted: failed to read robot pose from TF.')
            self._publish_centration_flag(False)
            self._centering_in_progress = False
            return
        cur_x, cur_y, cur_yaw = pose

        # 1) First try: global approach to nearest point to center with yaw within allowed band.
        approach_x, approach_y, approach_yaw = self._compute_allowed_approach_goal(cur_x, cur_y, cur_yaw)
        self.get_logger().info(
            'Initial global approach goal (map frame): '
            f'x={approach_x:.3f}, y={approach_y:.3f}, yaw={math.degrees(approach_yaw):.1f}deg'
        )
        self._send_graceful_goal_and_wait(approach_x, approach_y, approach_yaw)

        # Then try final target in map frame.
        self._send_graceful_goal_and_wait(self._target_x, self._target_y, self.yaw)

        cycle = 0
        while rclpy.ok() and (cycle < self.max_cycles):
            if not self._wait_for_robot_pose(timeout_sec=1.0):
                self.get_logger().warn(
                    f'No TF {self.frame_id}->{self.base_frame_id} while centering. Waiting...'
                )
                continue

            pose = self._get_robot_pose_from_tf()
            if pose is None:
                continue
            cur_x, cur_y, cur_yaw = pose
            cycle += 1

            dx = self._target_x - cur_x
            dy = self._target_y - cur_y
            pos_err = math.hypot(dx, dy)
            yaw_err = abs(normalize_angle(self.yaw - cur_yaw))
            longitudinal_err, lateral_err = self._compute_longitudinal_lateral_errors(
                cur_yaw, dx, dy
            )

            self.get_logger().info(
                f'Cycle {cycle}: robot=({cur_x:.3f},{cur_y:.3f}) yaw={math.degrees(cur_yaw):.1f} '
                f'pos_err={pos_err*100:.1f}cm yaw_err={math.degrees(yaw_err):.2f}deg '
                f'long_err={longitudinal_err*100:.1f}cm lat_err={lateral_err*100:.1f}cm'
            )

            if self._within_tolerance(cur_x, cur_y, cur_yaw):
                self.get_logger().info('Centered: within 0.5cm and 3deg. Done.')
                self._publish_centration_flag(False)
                self._centering_in_progress = False
                return

            # ТЗ: глобальный угол не должен выходить за допустимый диапазон.
            if not self._yaw_in_allowed_band(cur_yaw):
                self._recover_yaw(cur_x, cur_y, cur_yaw)
                continue

            # 2) If still not enough: retreat and move forward along longitudinal axis.
            if abs(longitudinal_err) <= self.longitudinal_trigger_m:
                self._send_graceful_goal_and_wait(self._target_x, self._target_y, self.yaw)
                continue

            retreat_x, retreat_y, retreat_yaw, d = self._compute_retreat_point(
                cur_x, cur_y, cur_yaw, longitudinal_err
            )
            self.get_logger().info(f'Retreat step: {d:.3f}m -> ({retreat_x:.3f},{retreat_y:.3f})')
            self._send_graceful_goal_and_wait(retreat_x, retreat_y, retreat_yaw)

            if not self._wait_for_robot_pose(timeout_sec=1.0):
                continue
            pose = self._get_robot_pose_from_tf()
            if pose is None:
                continue
            cur_x, cur_y, cur_yaw = pose
            if not self._yaw_in_allowed_band(cur_yaw):
                self._recover_yaw(cur_x, cur_y, cur_yaw)
                continue

            fwd_x, fwd_y, fwd_yaw = self._compute_forward_point(cur_x, cur_y, cur_yaw, d)
            self.get_logger().info(f'Forward step: {d:.3f}m -> ({fwd_x:.3f},{fwd_y:.3f})')
            self._send_graceful_goal_and_wait(fwd_x, fwd_y, fwd_yaw)

            # then retry direct target with fixed final yaw
            self._send_graceful_goal_and_wait(self._target_x, self._target_y, self.yaw)

        self.get_logger().warn('Centering finished by max_cycles without full tolerance.')
        self._publish_centration_flag(False)
        self._centering_in_progress = False

    def run(self):
        self.get_logger().info(
            f'Waiting for /centration_flag=True and TF {self.frame_id}->{self.base_frame_id} ...'
        )
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._centering_requested and not self._centering_in_progress:
                self._run_centering_session()

def main():
    rclpy.init()
    node = IraypleNavigator()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import math
import threading

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import FollowPath, NavigateToPose
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion

from irayple.msg import Irayple

TARGET_ANGLE = 270.0

def yaw_to_quaternion(yaw_rad: float) -> Quaternion:
    q = Quaternion()
    half = 0.5 * float(yaw_rad)
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q

class SimpleCenteringNavigator(Node):
    def __init__(self) -> None:
        super().__init__("simple_centering_navigator")

        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("min_goal_distance_m", 0.005)
        self.declare_parameter("epselon_yaw_deg", 30.0)
        self.declare_parameter("centering_action_name", "/centration_navigate_to_pose")
        self.declare_parameter(
            "behavior_tree_xml",
            "/home/ubuntu/ros2_ws/src/navigation/behavior_trees/navigate_to_pose_precise.xml",
        )

        self._centering_active = False

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._nav_client = ActionClient(self, NavigateToPose, "/graceful_navigate_to_pose")
        self._centering_server = ActionServer(
            self,
            NavigateToPose,
            str(self.get_parameter("centering_action_name").value),
            execute_callback=self._execute_centering_action,
            goal_callback=self._on_centering_goal,
            cancel_callback=self._on_centering_cancel,
        )
        self._centration_flag_pub = self.create_publisher(Bool, "/centration_flag", 10)
        self.create_subscription(Bool, "/centration_flag", self._on_flag, 10)
        self.create_subscription(Irayple, "/irayple", self._on_irayple, 10)

        self.get_logger().info("Centering Navigator started")

    def _on_irayple(self, msg: Irayple):
        self._last_irayple_xy_m = (msg.x / 10000.0, msg.y / 10000.0)

    def _on_flag(self, msg: Bool):
        if not msg.data:
            return
        with self._active_lock:
            if self._centering_active:
                self.get_logger().warn("Centering already active, ignore repeated True flag")
                return
            self._centering_active = True
        self.get_logger().info("Centration flag received, starting centering...")
        self._run_from_flag()

    def _publish_flag(self, value: bool):
        self._centration_flag_pub.publish(Bool(data=bool(value)))

    def _get_robot_pose(self):
        global_frame = str(self.get_parameter("global_frame").value)
        base_frame = str(self.get_parameter("base_frame").value)
        timeout = Duration(seconds=1.5)
        if not self._tf_buffer.can_transform(global_frame, base_frame, Time(), timeout):
            self.get_logger().error(f"TF unavailable: {global_frame}->{base_frame}")
            return None
        tf = self._tf_buffer.lookup_transform(global_frame, base_frame, Time())
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        _, _, yaw = euler_from_quaternion(
            [tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w]
        )
        return x, y, yaw

    def pose_of_qr(self):
        if self._last_irayple_xy_m is None:
            self.get_logger().error("No /irayple data, cannot center from flag mode")
            return None
        pose = self._get_robot_pose()
        if pose is None:
            return None
        robot_x, robot_y, robot_yaw = pose
        qr_x_cam, qr_y_cam = self._last_irayple_xy_m
        target_yaw = math.radians(float(self.get_parameter("target_yaw_deg").value))
        cos_y = math.cos(robot_yaw)
        sin_y = math.sin(robot_yaw)
        target_x = robot_x + (qr_x_cam * cos_y - qr_y_cam * sin_y)
        target_y = robot_y + (qr_x_cam * sin_y + qr_y_cam * cos_y)
        return target_x, target_y, target_yaw

    def _build_nav_goal(self, x: float, y: float, yaw: float):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = str(self.get_parameter("global_frame").value)
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation = yaw_to_quaternion(yaw)
        goal.behavior_tree = str(self.get_parameter("behavior_tree_xml").value)
        return goal

    def _send_nav_goal_and_wait(self, x: float, y: float, yaw: float):
        """ build global path."""
        interp_step_m = 0.10

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server unavailable")
            return GoalStatus.STATUS_ABORTED

        start_pose = self._get_robot_pose()
        if start_pose is None:
            return GoalStatus.STATUS_ABORTED
        sx, sy, _ = start_pose

        total_dist = math.hypot(x - sx, y - sy)
        steps = max(1, int(math.ceil(total_dist / interp_step_m)))

        def _send_one_goal(gx: float, gy: float, gyaw: float):
            done_event = threading.Event()
            status_holder = {"status": GoalStatus.STATUS_UNKNOWN}

            def _on_result(result_future):
                try:
                    status_holder["status"] = result_future.result().status
                except Exception as exc:
                    self.get_logger().error(f"Result callback failed: {exc}")
                    status_holder["status"] = GoalStatus.STATUS_ABORTED
                done_event.set()

            def _on_goal_response(send_future):
                try:
                    gh = send_future.result()
                except Exception as exc:
                    self.get_logger().error(f"Send goal failed: {exc}")
                    status_holder["status"] = GoalStatus.STATUS_ABORTED
                    done_event.set()
                    return
                if not gh.accepted:
                    self.get_logger().error("Goal rejected by graceful planner")
                    status_holder["status"] = GoalStatus.STATUS_ABORTED
                    done_event.set()
                    return
                gh.get_result_async().add_done_callback(_on_result)

            self._nav_client.send_goal_async(self._build_nav_goal(gx, gy, gyaw)).add_done_callback(_on_goal_response)

            done_event.wait()
            return status_holder["status"]

        self.get_logger().info(f"Interpolated straight plan to qr")

        for i in range(1, steps + 1):
            t = float(i) / float(steps)
            gx = sx + (x - sx) * t
            gy = sy + (y - sy) * t
            status = _send_one_goal(gx, gy, yaw)
            if status != GoalStatus.STATUS_SUCCEEDED:
                return status

        return GoalStatus.STATUS_SUCCEEDED

    def _run_from_flag(self):
        while self._centering_active:
            qr_x, qr_y, qr_yaw = self.pose_of_qr()
            if qr_x is None:
                self.get_logger().error("No qr pose, cannot center")
                return

            robot_pose = self._get_robot_pose()
            if robot_pose is None:
                self.get_logger().error("No robot pose, cannot center")
                return
            rx, ry, ryaw = robot_pose

            if abs(qr_yaw - TARGET_ANGLE) > 180.0:
                self.target_angle = TARGET_ANGLE + 180.0
            else:
                self.target_angle = TARGET_ANGLE
            self._send_nav_goal_and_wait(rx, ry, self.target_angle)
            # посчитаем продолное и поперечное отклонение от цели
            dx = qr_x - rx
            dy = qr_y - ry

            # Продольное отклонение:
            delta_longitudinal = dx * math.cos(qr_yaw) + dy * math.sin(qr_yaw)
            # Поперечное отклонение:
            delta_lateral = -dx * math.sin(qr_yaw) + dy * math.cos(qr_yaw)
            # если delta_longitudinal > 0, то цель впереди
            # если delta_longitudinal < 0, то цель позади
            # если delta_lateral > 0, то цель справа
            # если delta_lateral < 0, то цель слева
            
            if delta_lateral < self.get_parameter("epselon_lateral_m").value:
                status = self._send_nav_goal_and_wait(qr_x, qr_y, self.target_angle)
                if status != GoalStatus.STATUS_SUCCEEDED:
                    self.get_logger().error("Failed to navigate to qr")
                    return
            else:
                if delta_lateral > 0:
                    # поворачиваемся на угол чтоб ехвть назад
                    status = self._send_nav_goal_and_wait(rx, ry, (self.target_angle - self.get_parameter("epselon_lateral_m").value))
                    back_lateral = delta_lateral
                    back_longitudinal = back_lateral / math.tan(self.target_angle - math.radians(self.get_parameter("epselon_yaw_deg").value))
                else:
                    # поворачиваемся на угол чтоб ехвть назад
                    status = self._send_nav_goal_and_wait(rx, ry, (self.target_angle + self.get_parameter("epselon_lateral_m").value))
                    back_lateral = -delta_lateral
                    back_longitudinal = back_lateral / math.tan(self.target_angle + math.radians(self.get_parameter("epselon_yaw_deg").value))

                if status != GoalStatus.STATUS_SUCCEEDED:
                    self.get_logger().error("Failed to turning to go back")
                    return
                # едем назад
                status = self._send_nav_goal_and_wait(rx + back_longitudinal, ry + back_lateral, self.target_angle)
                if status != GoalStatus.STATUS_SUCCEEDED:
                    self.get_logger().error("Failed go back")
                    return
                # едем к цели
                status = self._send_nav_goal_and_wait(rx, ry, self.target_angle)
                if status != GoalStatus.STATUS_SUCCEEDED:
                    self.get_logger().error("Failed to navigate to qr")
                    return
                if (abs(ryaw - self.target_angle) > self.get_parameter("epselon_yaw_deg").value
                and abs(qr_x - rx) > self.get_parameter("epselon_longitudinal_m").value
                and abs(qr_y - ry) > self.get_parameter("epselon_lateral_m").value): # если не достигли цели, то продолжаем центровку
                    continue
                else:
                    # завершаем центровку
                    self._finish(True)
                return

    def _finish(self, success: bool):
        with self._active_lock:
            self._centering_active = False
        self._publish_flag(False)
        self.get_logger().info(f"Centration finished, success={success}")


def main(args=None) -> None:
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


if __name__ == "__main__":
    main()
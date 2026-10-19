#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""armplus / armpro 共用的 MoveIt/ROS 通信层。"""

import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from agx_arm_msgs.msg import AgxArmStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
)
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

MOVEIT_ERROR_TEXT = {
    1: "SUCCESS",
    -1: "PLANNING_FAILED",
    -2: "INVALID_MOTION_PLAN",
    -3: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
    -4: "CONTROL_FAILED",
    -5: "UNABLE_TO_AQUIRE_SENSOR_DATA",
    -6: "TIMED_OUT",
    -7: "PREEMPTED",
    -10: "START_STATE_IN_COLLISION",
    -11: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
    -12: "GOAL_IN_COLLISION",
    -13: "GOAL_VIOLATES_PATH_CONSTRAINTS",
    -14: "GOAL_CONSTRAINTS_VIOLATED",
    -15: "INVALID_GROUP_NAME",
    -16: "INVALID_GOAL_CONSTRAINTS",
    -17: "INVALID_ROBOT_STATE",
    -18: "INVALID_LINK_NAME",
    -21: "FRAME_TRANSFORM_FAILURE",
    -22: "COLLISION_CHECKING_UNAVAILABLE",
    -23: "ROBOT_STATE_STALE",
    -24: "SENSOR_INFO_STALE",
    -25: "COMMUNICATION_FAILURE",
    -26: "START_STATE_INVALID",
    -27: "GOAL_STATE_INVALID",
    -28: "UNRECOGNIZED_GOAL_TYPE",
    -31: "NO_IK_SOLUTION",
}

FOLLOW_ERROR_TEXT = {
    0: "SUCCESSFUL",
    -1: "INVALID_GOAL",
    -2: "INVALID_JOINTS",
    -3: "OLD_HEADER_TIMESTAMP",
    -4: "PATH_TOLERANCE_VIOLATED",
    -5: "GOAL_TOLERANCE_VIOLATED",
}


def to_duration(seconds: float) -> Duration:
    seconds = max(0.0, float(seconds))
    sec = int(seconds)
    nsec = int(round((seconds - sec) * 1e9))
    if nsec >= 1_000_000_000:
        sec += 1
        nsec -= 1_000_000_000
    return Duration(sec=sec, nanosec=nsec)


def duration_seconds(duration: Duration) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1e-9


class MoveItArmNode(Node):
    """反馈订阅 + MoveIt Action/Service 客户端。"""

    def __init__(self, joint_count: int = 6):
        super().__init__("arm_moveit_cli")
        self.joint_names = [f"joint{i}" for i in range(1, joint_count + 1)]
        self.current_rad = None
        self.latest_rad = None
        self.current_pose = None
        self.gripper_current = None  # (width_m, force_n)
        self.arm_status = None

        self.create_subscription(
            JointState, "/feedback/joint_states", self._on_joint_states, 10
        )
        self.create_subscription(
            PoseStamped, "/feedback/tcp_pose", self._on_tcp_pose, 10
        )
        self.create_subscription(
            AgxArmStatus, "/feedback/arm_status", self._on_arm_status, 10
        )

        self.arm_client = ActionClient(
            self, FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
        )
        self.gripper_client = ActionClient(
            self, FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory",
        )
        self.move_client = ActionClient(self, MoveGroup, "/move_action")
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.cart_client = self.create_client(
            GetCartesianPath, "/compute_cartesian_path"
        )

    # ── feedback ────────────────────────────────────────────────────
    def _on_joint_states(self, msg: JointState):
        positions = dict(zip(msg.name, msg.position))
        if all(name in positions for name in self.joint_names):
            self.current_rad = [positions[name] for name in self.joint_names]
            self.latest_rad = list(self.current_rad)

        if "gripper" in positions:
            index = msg.name.index("gripper")
            force = msg.effort[index] if index < len(msg.effort) else 0.0
            self.gripper_current = (positions["gripper"], force)

    def _on_tcp_pose(self, msg: PoseStamped):
        self.current_pose = msg

    def _on_arm_status(self, msg: AgxArmStatus):
        self.arm_status = msg

    def spin_for(self, seconds: float):
        deadline = time.time() + max(0.0, seconds)
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=min(0.05, max(0.0, deadline - time.time())))

    def wait_feedback(self, kind: str, timeout: float = 10.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if (kind == "j" and self.current_rad is not None) or (
                kind == "p" and self.current_pose is not None
            ):
                for _ in range(5):
                    rclpy.spin_once(self, timeout_sec=0.02)
                return True
        return False

    @property
    def current_deg(self):
        if self.current_rad is None:
            return None
        return [math.degrees(v) for v in self.current_rad]

    @property
    def latest_deg(self):
        if self.latest_rad is None:
            return None
        return [math.degrees(v) for v in self.latest_rad]

    def current_rpy_deg(self):
        if self.current_pose is None:
            return None
        from scipy.spatial.transform import Rotation as R

        q = self.current_pose.pose.orientation
        return R.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz", degrees=True)

    # ── capability detection ────────────────────────────────────────
    def control_stream_present(self) -> bool:
        """MoveIt 的 ros2_control 是否在持续发 /control/joint_states。"""
        try:
            return self.count_publishers("/control/joint_states") > 0
        except Exception:
            return False

    def move_group_follows_feedback(self) -> bool:
        """move_group 是否订阅真机反馈（follow:=true）。

        MoveIt 的 CurrentStateMonitor 运行在 move_group 的私有节点里，
        节点名可能是 move_group 或 move_group_private_<id>。
        """
        try:
            infos = self.get_subscriptions_info_by_topic("/feedback/joint_states")
            return any(
                info.node_name.startswith("move_group") for info in infos
            )
        except Exception:
            return False

    def wait_action(self, client: ActionClient, timeout: float = 1.0) -> bool:
        return client.wait_for_server(timeout_sec=timeout)

    def wait_service(self, client, timeout: float = 1.0) -> bool:
        return client.wait_for_service(timeout_sec=timeout)


# ── goal helpers ────────────────────────────────────────────────────
def build_joint_trajectory(joint_names, start_rad, target_rad,
                           speed_deg_s: float = 20.0):
    """构造 [当前点, 目标点] 关节轨迹，返回 (trajectory, duration_s)。"""
    deltas = [
        abs(math.degrees(t - s)) for s, t in zip(start_rad, target_rad)
    ]
    duration = max(1.0, max(deltas, default=0.0) / max(speed_deg_s, 1.0))

    traj = JointTrajectory()
    traj.joint_names = list(joint_names)

    p0 = JointTrajectoryPoint()
    p0.positions = [float(v) for v in start_rad]
    p0.time_from_start = to_duration(0.3)

    p1 = JointTrajectoryPoint()
    p1.positions = [float(v) for v in target_rad]
    p1.time_from_start = to_duration(0.3 + duration)

    traj.points = [p0, p1]
    return traj, duration


def build_gripper_trajectory(current_width, target_width,
                             speed_m_s: float = 0.05):
    # 没有夹爪反馈时，从目标值起步，避免先命令到 0 再张开。
    width = float(target_width) if current_width is None else float(current_width)
    delta = abs(float(target_width) - width)
    duration = max(1.0, delta / max(speed_m_s, 1e-3))

    traj = JointTrajectory()
    traj.joint_names = ["gripper"]

    p0 = JointTrajectoryPoint()
    p0.positions = [width]
    p0.time_from_start = to_duration(0.2)

    p1 = JointTrajectoryPoint()
    p1.positions = [float(target_width)]
    p1.time_from_start = to_duration(0.2 + duration)

    traj.points = [p0, p1]
    return traj, duration


def send_goal(node, client, goal, timeout: float):
    """发送 Action 目标并等待结果。

    返回 (goal_handle, result_wrapper, error_string)。
    """
    if not client.server_is_ready():
        return None, None, "动作服务器不可用"

    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future, timeout_sec=5.0)
    if not send_future.done():
        return None, None, "发送目标超时"
    goal_handle = send_future.result()
    if goal_handle is None or not goal_handle.accepted:
        return None, None, "目标被控制器拒绝"

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=timeout)
    if not result_future.done():
        return goal_handle, None, "等待执行结果超时"
    wrapped = result_future.result()
    if wrapped is None:
        return goal_handle, None, "未收到执行结果"
    return goal_handle, wrapped, None


def send_joint_trajectory(node, client, traj, timeout):
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = traj
    return send_goal(node, client, goal, timeout)


def joint_action_ok(wrapped):
    if wrapped is None:
        return False, "未收到执行结果"
    if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
        return False, f"Action 状态异常（status={wrapped.status}）"
    code = int(wrapped.result.error_code)
    if code != 0:
        return False, (
            f"控制器返回 error_code={code} "
            f"({FOLLOW_ERROR_TEXT.get(code, '未知')})"
        )
    return True, ""


def move_action_ok(wrapped):
    if wrapped is None:
        return False, "未收到执行结果"
    if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
        return False, f"Action 状态异常（status={wrapped.status}）"
    code = int(wrapped.result.error_code.val)
    if code != MoveItErrorCodes.SUCCESS:
        return False, (
            f"MoveIt error_code={code} "
            f"({MOVEIT_ERROR_TEXT.get(code, '未知')})"
        )
    return True, ""


def trajectory_duration(traj: JointTrajectory) -> float:
    if not traj.points:
        return 0.0
    return duration_seconds(traj.points[-1].time_from_start)


# ── IK ──────────────────────────────────────────────────────────────
def compute_ik(node: MoveItArmNode, position_m, quat, seed_rad=None,
               timeout_s: float = 1.0):
    """返回 (joint_positions_rad, error_string)。"""
    req = GetPositionIK.Request()
    ik = req.ik_request
    ik.group_name = "arm"
    ik.ik_link_name = "tcp_link"
    ik.pose_stamped.header.frame_id = "base_link"
    ik.pose_stamped.header.stamp = node.get_clock().now().to_msg()
    ik.pose_stamped.pose.position.x = float(position_m[0])
    ik.pose_stamped.pose.position.y = float(position_m[1])
    ik.pose_stamped.pose.position.z = float(position_m[2])
    ik.pose_stamped.pose.orientation.x = float(quat[0])
    ik.pose_stamped.pose.orientation.y = float(quat[1])
    ik.pose_stamped.pose.orientation.z = float(quat[2])
    ik.pose_stamped.pose.orientation.w = float(quat[3])
    ik.avoid_collisions = False
    ik.timeout = to_duration(timeout_s)

    if seed_rad is not None:
        ik.robot_state.joint_state.header.stamp = node.get_clock().now().to_msg()
        ik.robot_state.joint_state.name = list(node.joint_names)
        ik.robot_state.joint_state.position = [float(v) for v in seed_rad]

    future = node.ik_client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s + 3.0)
    if not future.done() or future.result() is None:
        return None, "IK 服务调用超时"

    resp = future.result()
    code = int(resp.error_code.val)
    if code != MoveItErrorCodes.SUCCESS:
        return None, (
            f"IK 无解（error_code={code}, "
            f"{MOVEIT_ERROR_TEXT.get(code, '未知')}）"
        )

    joint_state = resp.solution.joint_state
    positions = dict(zip(joint_state.name, joint_state.position))
    missing = [name for name in node.joint_names if name not in positions]
    if missing:
        return None, f"IK 返回缺少关节: {', '.join(missing)}"
    return [float(positions[name]) for name in node.joint_names], None


# ── Cartesian path ──────────────────────────────────────────────────
def compute_cartesian(node: MoveItArmNode, target_pose: Pose,
                      start_rad, max_step: float = 0.005,
                      vel: float = 0.1, acc: float = 0.1):
    """返回 (RobotTrajectory, fraction, error_string)。"""
    req = GetCartesianPath.Request()
    req.header.frame_id = "base_link"
    req.header.stamp = node.get_clock().now().to_msg()
    req.start_state.joint_state.header.stamp = req.header.stamp
    req.start_state.joint_state.name = list(node.joint_names)
    req.start_state.joint_state.position = [float(v) for v in start_rad]
    req.group_name = "arm"
    req.link_name = "tcp_link"
    req.waypoints = [target_pose]
    req.max_step = float(max_step)
    req.jump_threshold = 0.0
    req.avoid_collisions = False
    req.max_velocity_scaling_factor = float(vel)
    req.max_acceleration_scaling_factor = float(acc)

    future = node.cart_client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=15.0)
    if not future.done() or future.result() is None:
        return None, 0.0, "Cartesian 路径服务调用超时"

    resp = future.result()
    code = int(resp.error_code.val)
    if code != MoveItErrorCodes.SUCCESS:
        return None, float(resp.fraction), (
            f"Cartesian 路径失败（error_code={code}, "
            f"{MOVEIT_ERROR_TEXT.get(code, '未知')}）"
        )
    return resp.solution, float(resp.fraction), None


# ── MoveGroup planning ──────────────────────────────────────────────
def build_move_goal(node: MoveItArmNode, target_rad, vel: float = 0.1,
                    acc: float = 0.1, planner: str = "",
                    plan_time: float = 5.0):
    goal = MoveGroup.Goal()
    req = goal.request
    req.group_name = "arm"
    req.num_planning_attempts = 5
    req.allowed_planning_time = float(plan_time)
    req.max_velocity_scaling_factor = float(vel)
    req.max_acceleration_scaling_factor = float(acc)
    req.planner_id = planner or ""

    constraints = Constraints()
    for name, position in zip(node.joint_names, target_rad):
        jc = JointConstraint()
        jc.joint_name = name
        jc.position = float(position)
        # 0.01 rad ≈ 0.57°，对 2° 的小步来说误差太大；收紧到约 0.06°。
        jc.tolerance_above = 0.001
        jc.tolerance_below = 0.001
        jc.weight = 1.0
        constraints.joint_constraints.append(jc)
    req.goal_constraints = [constraints]

    goal.planning_options.plan_only = False
    goal.planning_options.replan = False
    return goal

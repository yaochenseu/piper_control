#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""armplus：MoveIt 控制器直连（Route A）。

用法与 arm 一致：
    armplus j <j1..j6> [夹爪比例] [选项]
    armplus p <x> <y> <z> [rx ry rz] [夹爪比例] [选项]

特点：
    - 关节轨迹直接发给 MoveIt 的 /arm_controller/follow_joint_trajectory
    - 笛卡尔目标先用 /compute_ik 求关节角，再走控制器
    - 不做 MoveIt 规划（需要规划请用 armpro）
    - MoveIt 未启动时，确认后可降级为 arm 直连驱动方式

单位：关节角/姿态=度，位置=毫米，夹爪比例=0~1。
"""

import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R

import arm_common as C
import moveit_common as M


# ── 启动前的能力检测 ────────────────────────────────────────────────
def decide_backend(node, args, need_gripper):
    """返回 'moveit' 或 'arm'；无法安全执行时退出。"""
    while True:
        node.spin_for(0.6)
        missing = []

        if not node.wait_action(node.arm_client, 2.0):
            missing.append("/arm_controller/follow_joint_trajectory")
        if need_gripper and not node.wait_action(node.gripper_client, 2.0):
            missing.append("/gripper_controller/follow_joint_trajectory")
        if args.mode == "p" and not node.wait_service(node.ik_client, 2.0):
            missing.append("/compute_ik")
        if args.mode == "p" and args.linear and not node.wait_service(
            node.cart_client, 1.0
        ):
            missing.append("/compute_cartesian_path")

        if not missing:
            return "moveit"

        print("⚠️ MoveIt 能力不足，缺少：")
        for item in missing:
            print(f"   - {item}")

        stream = node.control_stream_present()
        if stream:
            print("   检测到 /control/joint_states 正在发布"
                  "（MoveIt 控制器已启动）。")
            print("   此时不能降级为 arm 直连，"
                  "否则指令会被 200 Hz 控制流覆盖。")
            if C.prompt_retry():
                continue
            print("❌ 未执行：MoveIt 状态不完整，拒绝降级。")
            sys.exit(2)

        reason = ("未检测到 MoveIt 控制器，"
                  "且 /control/joint_states 没有发布者。")
        if C.confirm_degrade(args, reason):
            node.spin_for(0.6)
            if node.control_stream_present():
                print("❌ 未执行：确认后检测到控制流出现，取消降级。")
                sys.exit(2)
            return "arm"
        print("❌ 未执行：已取消降级。")
        sys.exit(2)


# ── 关节模式 ────────────────────────────────────────────────────────
def run_joint(args, node, arm_tokens, gripper_token):
    print("等待 /feedback/joint_states ...")
    if not node.wait_feedback("j"):
        print("超时：没有收到完整关节反馈。先确认驱动已启动且已使能。")
        print("❌ 失败：未收到关节反馈")
        sys.exit(2)

    names = node.joint_names
    current_deg = list(node.current_deg)

    target_deg = []
    kept = []
    for i, token in enumerate(arm_tokens):
        value, keep = C.resolve_token(token, current_deg[i])
        target_deg.append(value)
        kept.append(keep)

    gripper_ratio = C.resolve_gripper(gripper_token, node.gripper_current)
    C.check_gripper(gripper_ratio, args.force)

    limits_deg = C.load_joint_limits_deg(args.robot)
    out_of_limits = [
        (i, name) for i, name in enumerate(names)
        if not kept[i]
        and not (limits_deg[name][0] <= target_deg[i] <= limits_deg[name][1])
    ]
    if out_of_limits:
        if not args.clamp:
            print(f"目标角超出 {args.robot} 关节限位：")
            for i, name in out_of_limits:
                low, high = limits_deg[name]
                print(
                    f"  {name}: 目标 {target_deg[i]:+.1f}°，限位 "
                    f"[{low:+.1f}°, {high:+.1f}°]"
                )
            print("确认后加 -c 自动夹紧，或修改目标角。")
            sys.exit(4)
        for i, name in out_of_limits:
            low, high = limits_deg[name]
            clamped = min(max(target_deg[i], low), high)
            print(
                f"警告: {name} 目标 {target_deg[i]:+.1f}° 超出限位 "
                f"[{low:+.1f}°, {high:+.1f}°]，已夹紧到 {clamped:+.1f}°"
            )
            target_deg[i] = clamped

    delta_deg = [t - c for t, c in zip(target_deg, current_deg)]

    print("\n当前关节角（度）:")
    for name, value in zip(names, current_deg):
        print(f"  {name}: {value:+.2f}°")
    print("\n目标关节角（度）:")
    for name, value, d, keep in zip(names, target_deg, delta_deg, kept):
        suffix = "   (保持当前)" if keep else f"   (变化 {d:+.2f}°)"
        print(f"  {name}: {value:+.2f}°{suffix}")

    gripper_width = None
    if gripper_ratio is not None:
        gripper_width = C.GRIPPER_MAX_WIDTH * gripper_ratio
    C.print_gripper(node.gripper_current, gripper_ratio, args.force,
                    gripper_token)

    if args.max_delta is not None:
        too_large = [
            (name, d) for name, d in zip(names, delta_deg)
            if abs(d) > args.max_delta
        ]
        if too_large:
            print(
                f"\n拒绝执行：以下关节单次变化超过 {args.max_delta:.1f}°"
                "（你显式设置了 --md）："
            )
            for name, d in too_large:
                print(f"  {name}: {d:+.2f}°")
            print("确认安全后可调大或去掉 --md。")
            sys.exit(3)

    print(f"\n后端: MoveIt 控制器直连，轨迹速度 {args.speed:.1f}°/s")

    if args.preview:
        print("\n仅预演，未发送指令。去掉 -n 即直接执行。")
        return

    target_rad = [math.radians(v) for v in target_deg]
    traj, duration = M.build_joint_trajectory(
        names, node.current_rad, target_rad, args.speed
    )
    _gh, wrapped, err = M.send_joint_trajectory(
        node, node.arm_client, traj, timeout=duration + 20.0
    )

    if err:
        action_ok, action_msg = False, err
    else:
        action_ok, action_msg = M.joint_action_ok(wrapped)

    gripper_ok = True
    if gripper_width is not None:
        cur_width = (
            node.gripper_current[0] if node.gripper_current is not None else None
        )
        gtraj, gdur = M.build_gripper_trajectory(cur_width, gripper_width)
        _gh2, gwrapped, gerr = M.send_joint_trajectory(
            node, node.gripper_client, gtraj, timeout=gdur + 15.0
        )
        if gerr:
            gripper_ok, gripper_msg = False, gerr
        else:
            gripper_ok, gripper_msg = M.joint_action_ok(gwrapped)
        if gripper_ok:
            print(
                f"夹爪指令已发送：{gripper_width * 1000:.0f} mm "
                f"({gripper_ratio * 100:.0f}%)"
            )
        elif action_ok:
            action_ok = False
            action_msg = f"夹爪：{gripper_msg}"

    C.monitor_joint(node, target_deg, args.wait, action_ok, action_msg)


# ── 笛卡尔模式 ──────────────────────────────────────────────────────
def run_pose(args, node, position_tokens, rpy_tokens, gripper_token):
    print("等待 /feedback/tcp_pose ...")
    if not node.wait_feedback("p"):
        print("超时：没有收到 TCP 位姿反馈。先确认驱动已启动且已使能。")
        print("❌ 失败：未收到 TCP 位姿反馈")
        sys.exit(2)

    p = node.current_pose.pose.position
    current_pos = [p.x * 1000.0, p.y * 1000.0, p.z * 1000.0]
    current_rpy = node.current_rpy_deg()

    position = []
    kept_pos = []
    for i, token in enumerate(position_tokens):
        value, keep = C.resolve_token(token, current_pos[i])
        position.append(value)
        kept_pos.append(keep)

    if rpy_tokens is None:
        rpy_deg = list(current_rpy)
        kept_rpy = [True, True, True]
    else:
        rpy_deg = []
        kept_rpy = []
        for i, token in enumerate(rpy_tokens):
            value, keep = C.resolve_token(token, current_rpy[i])
            rpy_deg.append(value)
            kept_rpy.append(keep)

    gripper_ratio = C.resolve_gripper(gripper_token, node.gripper_current)
    C.check_gripper(gripper_ratio, args.force)

    pos_delta = [t - c for t, c in zip(position, current_pos)]
    r_target = R.from_euler("xyz", rpy_deg, degrees=True)
    q = node.current_pose.pose.orientation
    r_current = R.from_quat([q.x, q.y, q.z, q.w])
    rpy_delta = (r_target * r_current.inv()).as_euler("xyz", degrees=True)

    print("\n当前 TCP 位姿:")
    print(
        f"  x: {current_pos[0]:+.1f} mm   "
        f"y: {current_pos[1]:+.1f} mm   z: {current_pos[2]:+.1f} mm"
    )
    print(
        f"  rx: {current_rpy[0]:+.2f}°   "
        f"ry: {current_rpy[1]:+.2f}°   rz: {current_rpy[2]:+.2f}°"
    )
    print("\n目标 TCP 位姿:")
    print(
        f"  x: {position[0]:+.1f} mm   "
        f"y: {position[1]:+.1f} mm   z: {position[2]:+.1f} mm"
    )
    print(
        f"  rx: {rpy_deg[0]:+.2f}°   "
        f"ry: {rpy_deg[1]:+.2f}°   rz: {rpy_deg[2]:+.2f}°"
    )
    print(
        "  Δ位置: "
        f"[{pos_delta[0]:+.1f} {pos_delta[1]:+.1f} {pos_delta[2]:+.1f}] mm"
    )
    print(
        "  Δ姿态: "
        f"[{rpy_delta[0]:+.2f} {rpy_delta[1]:+.2f} {rpy_delta[2]:+.2f}]°"
    )
    kept_names = [
        name for name, keep in zip(
            ["x", "y", "z", "rx", "ry", "rz"],
            kept_pos + kept_rpy,
        ) if keep
    ]
    if kept_names:
        print(f"  (保持当前: {', '.join(kept_names)})")
    print(f"\n运动方式: {'Cartesian 直线（-l）' if args.linear else 'IK + 关节轨迹'}")

    C.print_gripper(node.gripper_current, gripper_ratio, args.force,
                    gripper_token)

    target_pose = Pose()
    target_pose.position.x = position[0] / 1000.0
    target_pose.position.y = position[1] / 1000.0
    target_pose.position.z = position[2] / 1000.0
    quat = r_target.as_quat()
    target_pose.orientation.x = float(quat[0])
    target_pose.orientation.y = float(quat[1])
    target_pose.orientation.z = float(quat[2])
    target_pose.orientation.w = float(quat[3])

    print("\n调用 /compute_ik ...")
    ik_rad, ik_err = M.compute_ik(
        node,
        [target_pose.position.x, target_pose.position.y,
         target_pose.position.z],
        quat,
        node.current_rad,
    )
    if ik_err:
        print("可尝试：调整目标位姿，或改用 arm p（固件 IK）。")
        print(f"❌ 失败：{ik_err}")
        return

    ik_deg = [math.degrees(v) for v in ik_rad]
    print("IK 解（度）:")
    for name, value in zip(node.joint_names, ik_deg):
        print(f"  {name}: {value:+.2f}°")

    print(f"\n后端: MoveIt 控制器直连，轨迹速度 {args.speed:.1f}°/s")

    if args.preview:
        print("\n仅预演，未发送指令。去掉 -n 即直接执行。")
        return

    if args.linear:
        solution, fraction, cart_err = M.compute_cartesian(
            node, target_pose, node.current_rad
        )
        if cart_err:
            action_ok, action_msg = False, cart_err
        elif fraction < 0.95:
            action_ok = False
            action_msg = f"Cartesian 路径只完成 {fraction * 100:.1f}%"
        else:
            print(f"Cartesian 路径完成度: {fraction * 100:.1f}%")
            traj = solution.joint_trajectory
            duration = M.trajectory_duration(traj)
            _gh, wrapped, err = M.send_joint_trajectory(
                node, node.arm_client, traj, timeout=duration + 20.0
            )
            action_ok, action_msg = (
                (False, err) if err else M.joint_action_ok(wrapped)
            )
    else:
        traj, duration = M.build_joint_trajectory(
            node.joint_names, node.current_rad, ik_rad, args.speed
        )
        _gh, wrapped, err = M.send_joint_trajectory(
            node, node.arm_client, traj, timeout=duration + 20.0
        )
        action_ok, action_msg = (
            (False, err) if err else M.joint_action_ok(wrapped)
        )

    gripper_width = None
    if gripper_ratio is not None:
        gripper_width = C.GRIPPER_MAX_WIDTH * gripper_ratio

    if gripper_width is not None:
        cur_width = (
            node.gripper_current[0] if node.gripper_current is not None else None
        )
        gtraj, gdur = M.build_gripper_trajectory(cur_width, gripper_width)
        _gh2, gwrapped, gerr = M.send_joint_trajectory(
            node, node.gripper_client, gtraj, timeout=gdur + 15.0
        )
        if gerr:
            gripper_ok, gripper_msg = False, gerr
        else:
            gripper_ok, gripper_msg = M.joint_action_ok(gwrapped)
        if gripper_ok:
            print(
                f"夹爪指令已发送：{gripper_width * 1000:.0f} mm "
                f"({gripper_ratio * 100:.0f}%)"
            )
        elif action_ok:
            action_ok = False
            action_msg = f"夹爪：{gripper_msg}"

    C.monitor_pose(node, position, rpy_deg, args.wait, action_ok, action_msg)


# ── 入口 ────────────────────────────────────────────────────────────
def main():
    parser = C.build_common_parser(
        "armplus",
        "MoveIt 控制器直连（Route A）：j=关节角（度），p=笛卡尔位姿（mm/度）",
    )
    parser.add_argument(
        "--speed", type=float, default=20.0,
        help="关节轨迹速度（度/秒），默认 20",
    )
    args = parser.parse_intermixed_args()
    C.finalize_args(parser, args)

    try:
        if args.mode == "j":
            arm_tokens, gripper_token = C.parse_joint_target(
                args.target, args.joints
            )
            position_tokens = rpy_tokens = None
        else:
            position_tokens, rpy_tokens, gripper_token = C.parse_pose_target(
                args.target
            )
            arm_tokens = None
    except ValueError as exc:
        print(f"参数错误：{exc}")
        sys.exit(1)

    need_gripper = (
        gripper_token is not None and gripper_token.lower() != "x"
    )

    rclpy.init()
    node = M.MoveItArmNode(args.joints)

    try:
        backend = decide_backend(node, args, need_gripper)
        if backend == "arm":
            node.destroy_node()
            rclpy.shutdown()
            C.exec_arm(args)

        try:
            if args.mode == "j":
                run_joint(args, node, arm_tokens, gripper_token)
            else:
                run_pose(args, node, position_tokens, rpy_tokens,
                         gripper_token)
        except ValueError as exc:
            print(f"参数错误：{exc}")
            sys.exit(1)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""armplus / armpro 共用的参数解析、限位检查和结果打印。

目标语法与 arm.py 完全一致：
    30      绝对目标
    x       保持当前
    x+10    当前值 +10
    x-10    当前值 -10

单位：关节角/姿态=度，位置=毫米，夹爪比例=0~1。
"""

import argparse
import math
import os
import sys

from scipy.spatial.transform import Rotation as R

GRIPPER_MAX_WIDTH = 0.1  # 米

ARM_STATUS_TEXT = {
    0: "正常",
    1: "急停",
    2: "无解（目标不可达）",
    3: "奇异点",
    4: "目标角度超限",
    5: "关节通信异常",
    6: "关节刹车未释放",
    7: "发生碰撞",
    8: "示教拖动超速",
    9: "关节状态异常",
    10: "其他异常",
    11: "示教记录中",
    12: "示教执行中",
    13: "示教暂停",
    14: "主控 NTC 过温",
    15: "释放电阻 NTC 过温",
}

MODE_TEXT = {
    0: "MOVE P",
    1: "MOVE J",
    2: "MOVE L",
    3: "MOVE C",
    4: "MOVE MIT",
    5: "MOVE CPV",
}


# ── 关节限位 ────────────────────────────────────────────────────────
def load_joint_limits_deg(robot: str) -> dict:
    """从 pyAgxArm 配置读取关节限位，并换算成度。不连接 CAN。"""
    from pyAgxArm import create_agx_arm_config

    config = create_agx_arm_config(robot=robot, comm="can", channel="can0")
    limits = config["joint_limits"]
    return {
        name: (math.degrees(low), math.degrees(high))
        for name, (low, high) in limits.items()
    }


# ── token 解析 ──────────────────────────────────────────────────────
def flatten(values):
    flat = []
    for value in values:
        flat.extend(v for v in str(value).replace(",", " ").split() if v)
    return flat


def is_valid_token(token: str) -> bool:
    t = token.lower()
    if t == "x":
        return True
    if len(t) >= 3 and t[0] == "x" and t[1] in "+-":
        try:
            float(t[1:])
            return True
        except ValueError:
            return False
    try:
        float(token)
        return True
    except ValueError:
        return False


def resolve_token(token: str, current: float):
    """返回 (value, keep_current)。"""
    t = token.lower()
    if t == "x":
        return current, True
    if len(t) >= 3 and t[0] == "x" and t[1] in "+-":
        return current + float(t[1:]), False
    return float(token), False


def check_tokens(tokens):
    for token in tokens:
        if not is_valid_token(token):
            raise ValueError(
                f"无效参数 '{token}'，请填数字、x（保持当前）、"
                "x+数值 或 x-数值"
            )


def parse_joint_target(values, joint_count: int):
    """返回 (臂关节 token 列表, 夹爪 token 或 None)。"""
    flat = flatten(values)
    if len(flat) not in (joint_count, joint_count + 1):
        raise ValueError(
            f"需要 {joint_count} 个臂关节角，或 {joint_count + 1} 个值"
            f"（最后一个为夹爪张开比例），实际收到 {len(flat)} 个: {flat}"
        )
    check_tokens(flat)
    return flat[:joint_count], (flat[-1] if len(flat) == joint_count + 1 else None)


def parse_pose_target(values):
    """返回 (position_tokens, rpy_tokens_or_None, gripper_token_or_None)。"""
    flat = flatten(values)
    if len(flat) not in (3, 6, 7):
        raise ValueError(
            "需要 3 个值（x y z）、6 个值（x y z rx ry rz）"
            "或 7 个值（再加夹爪比例），"
            f"实际收到 {len(flat)} 个: {flat}"
        )
    check_tokens(flat)
    position = flat[:3]
    rpy_deg = flat[3:6] if len(flat) >= 6 else None
    gripper_ratio = flat[6] if len(flat) == 7 else None
    return position, rpy_deg, gripper_ratio


def resolve_gripper(token, gripper_current):
    """夹爪 token -> 张开比例；x 或未提供返回 None。"""
    if token is None:
        return None
    t = token.lower()
    if t == "x":
        return None
    if len(t) >= 3 and t[0] == "x" and t[1] in "+-":
        if gripper_current is None:
            raise ValueError("无法计算夹爪相对目标：未收到夹爪反馈")
        current_ratio = gripper_current[0] / GRIPPER_MAX_WIDTH
        return current_ratio + float(t[1:])
    return float(token)


def check_gripper(ratio, force):
    if ratio is None:
        if force != 1.0:
            print("提示：夹爪未指定数值（未提供或为 x），-f 将被忽略")
        return
    if not (0.0 <= ratio <= 1.0):
        raise ValueError(
            f"夹爪张开比例必须在 0.0~1.0，当前 {ratio}"
            "（0=闭合，0.5=半开，1=全开）"
        )
    if not (0.5 <= force <= 3.0):
        raise ValueError(f"夹爪力必须在 0.5~3.0 N，当前 {force}")


def print_gripper(gripper_current, ratio, force, token, moveit=True):
    if ratio is None:
        if token is not None and token.lower() == "x":
            print("\n夹爪（第 7 轴）: 保持当前（不发送夹爪指令）")
        return
    width = GRIPPER_MAX_WIDTH * ratio
    if gripper_current is not None:
        cur_width, cur_force = gripper_current
        cur_ratio = cur_width / GRIPPER_MAX_WIDTH
        print(
            f"\n夹爪（第 7 轴）: 当前 {cur_width * 1000:.0f} mm "
            f"({cur_ratio * 100:.0f}%) / 反馈力 {cur_force:.2f} N"
            f"  ->  目标 {width * 1000:.0f} mm ({ratio * 100:.0f}%)"
        )
    else:
        print(
            f"\n夹爪（第 7 轴）: 目标 {width * 1000:.0f} mm "
            f"({ratio * 100:.0f}%)（未收到夹爪反馈）"
        )
    if moveit:
        print("  提示：MoveIt 夹爪控制器为位置接口，-f 不会生效，"
              "驱动使用默认约 1.0 N。")


# ── 参数解析 ────────────────────────────────────────────────────────
def build_common_parser(prog: str, description: str) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog=prog,
        description=description,
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "示例：\n"
            f"  {prog} j x x -30 x x x\n"
            f"  {prog} j x+10 x x x x x 0.5 -f 1.0\n"
            f"  {prog} p x x x+10\n"
            f"  {prog} p 96.9 0 300 -85 0 -90 -l\n"
            f"  {prog} j -n x x -30 x x x"
        ),
    )
    parser.add_argument(
        "mode", choices=["j", "p"], metavar="j|p",
        help="j=关节角控制，p=笛卡尔位姿控制",
    )
    parser.add_argument(
        "targets", nargs="*", metavar="目标",
        help="目标值，直接写在 j/p 后面；也可用 -t 指定",
    )
    parser.add_argument(
        "-t", "--target", dest="target_opt", nargs="+", default=None,
        help=(
            "目标值（可省略）。"
            "j 模式：6 个关节角（度）+ 可选夹爪比例；"
            "p 模式：x y z（mm）+ 可选 rx ry rz（度）+ 可选夹爪比例。"
            "支持 x / x+10 / x-10"
        ),
    )
    parser.add_argument(
        "-f", "--force", type=float, default=1.0,
        help="夹爪力（N），0.5~3.0，默认 1.0（MoveIt 控制器下不生效）",
    )
    parser.add_argument(
        "-n", "--no", "--dry-run", dest="preview", action="store_true",
        help="只预演，不发送指令（默认直接执行）",
    )
    parser.add_argument(
        "-l", "--linear", action="store_true",
        help="p 模式使用 MoveIt Cartesian 直线路径",
    )
    parser.add_argument(
        "-c", "--clamp", action="store_true",
        help="j 模式超限时自动夹紧到边界，否则拒绝执行",
    )
    parser.add_argument(
        "-w", "--wait", type=float, default=3.0,
        help="执行后等待/监控秒数，默认 3.0",
    )
    parser.add_argument(
        "--md", "--max-delta", dest="max_delta", type=float, default=None,
        help="j 模式可选：单次变化保护（度）",
    )
    parser.add_argument(
        "--jn", "--joints", dest="joints", type=int, default=6,
        help="关节数（Piper=6，Nero=7），默认 6",
    )
    parser.add_argument(
        "--rb", "--robot", dest="robot", default="piper_x",
        help="型号，用于读取关节限位（默认 piper_x）",
    )
    parser.add_argument(
        "-y", "--yes", action="store_true",
        help="MoveIt 不可用时自动确认降级为 arm 直连方式",
    )
    return parser


def finalize_args(parser, args):
    if args.target_opt is not None and args.targets:
        parser.error("目标值不要同时使用位置参数和 -t")
    args.target = args.target_opt if args.target_opt is not None else args.targets
    if not args.target:
        parser.error("缺少目标值（直接写在 j/p 后面，或用 -t）")
    if args.mode == "j" and args.linear:
        print("提示：j 模式不使用 -l，已忽略")
    return args


# ── 降级为 arm.py ──────────────────────────────────────────────────
def canonical_arm_argv(args):
    argv = [args.mode, *[str(v) for v in args.target]]
    if args.force != 1.0:
        argv += ["-f", str(args.force)]
    if args.preview:
        argv += ["-n"]
    if args.linear and args.mode == "p":
        argv += ["-l"]
    if args.clamp:
        argv += ["-c"]
    if args.wait != 3.0:
        argv += ["-w", str(args.wait)]
    if args.max_delta is not None:
        argv += ["--md", str(args.max_delta)]
    if args.joints != 6:
        argv += ["--jn", str(args.joints)]
    if args.robot != "piper_x":
        argv += ["--rb", args.robot]
    return argv


def exec_arm(args):
    arm_py = os.path.expanduser("~/agx_arm_ws/tools/arm.py")
    if not os.path.isfile(arm_py):
        print(f"❌ 找不到 {arm_py}")
        sys.exit(2)
    if any(getattr(args, name, None) is not None
           for name in ("speed", "vel", "acc", "planner")):
        print("  提示：降级后 --speed / --vel / --acc / --planner 不生效，"
              "速度由驱动 speed_percent 控制。")
    argv = [sys.executable, arm_py, *canonical_arm_argv(args)]
    print(f"→ 降级执行: {' '.join(argv[1:])}")
    # execv 不会刷新 Python 的缓冲区；管道输出时必须手动 flush。
    sys.stdout.flush()
    sys.stderr.flush()
    os.execv(sys.executable, argv)


def confirm_degrade(args, reason):
    print(f"⚠️ {reason}")
    if args.preview:
        print("  当前是 -n 预演，不会运动；将用 arm.py 预演。")
        return True
    if args.yes:
        print("  已指定 -y，自动降级为 arm 直连方式。")
        return True
    if not sys.stdin.isatty():
        print("  ❌ 当前不是交互终端；如确认降级请加 -y。")
        return False
    ans = input("  是否降级为 arm 直连驱动方式继续？[y/N]: ").strip().lower()
    return ans in ("y", "yes")


def prompt_retry():
    if not sys.stdin.isatty():
        return False
    while True:
        ans = input("  输入 r 重试，q 退出 [r/q]: ").strip().lower()
        if ans == "r":
            return True
        if ans == "q":
            return False


# ── 结果监控 ────────────────────────────────────────────────────────
def monitor_joint(node, target_deg, wait, action_ok=True, action_msg=""):
    node.spin_for(wait)

    actual_deg = node.latest_deg
    if actual_deg is None:
        print("\n未收到最终关节反馈")
        print("❌ 失败：未收到关节反馈")
        return

    err = [t - a for t, a in zip(target_deg, actual_deg)]
    actual = " ".join(f"{v:+.2f}" for v in actual_deg)
    error = " ".join(f"{v:+.2f}" for v in err)
    print(f"\n最终实际（度）: [{actual}]")
    print(f"最终误差（度）: [{error}]")

    if not action_ok:
        print(f"❌ 失败：{action_msg}")
        return

    if node.arm_status is None:
        print("❌ 失败：未收到 arm_status")
        return

    arm_code = node.arm_status.arm_status
    motion = node.arm_status.motion_status
    max_err = max(abs(v) for v in err)
    if arm_code != 0:
        print(
            f"❌ 失败：arm_status={arm_code} "
            f"({ARM_STATUS_TEXT.get(arm_code, '未知')})"
        )
    elif motion != 0:
        print(f"❌ 失败：未到达目标（motion_status={motion}）")
    elif max_err > 2.0:
        print(f"⚠️ 状态成功但误差偏大：最大关节误差 {max_err:.2f}°")
    else:
        print(f"✅ 成功：已到达目标（最大关节误差 {max_err:.2f}°）")


def monitor_pose(node, target_position, target_rpy_deg, wait,
                 action_ok=True, action_msg=""):
    node.spin_for(wait)

    if node.current_pose is None:
        print("\n未收到最终 TCP 位姿反馈")
        print("❌ 失败：未收到 TCP 位姿反馈")
        return

    p = node.current_pose.pose.position
    actual_rpy = node.current_rpy_deg()
    actual_pos = [p.x * 1000.0, p.y * 1000.0, p.z * 1000.0]

    pos_err = [t - a for t, a in zip(target_position, actual_pos)]
    r_target = R.from_euler("xyz", target_rpy_deg, degrees=True)
    q = node.current_pose.pose.orientation
    r_actual = R.from_quat([q.x, q.y, q.z, q.w])
    rpy_err = (r_target * r_actual.inv()).as_euler("xyz", degrees=True)

    print(
        "\n最终实际位置（mm）: "
        f"[{actual_pos[0]:+.1f} {actual_pos[1]:+.1f} {actual_pos[2]:+.1f}]"
    )
    print(
        "最终实际姿态（度）: "
        f"[{actual_rpy[0]:+.2f} {actual_rpy[1]:+.2f} {actual_rpy[2]:+.2f}]"
    )
    print(
        "最终位置误差（mm）: "
        f"[{pos_err[0]:+.1f} {pos_err[1]:+.1f} {pos_err[2]:+.1f}]"
    )
    print(
        "最终姿态误差（度）: "
        f"[{rpy_err[0]:+.2f} {rpy_err[1]:+.2f} {rpy_err[2]:+.2f}]"
    )

    if not action_ok:
        print(f"❌ 失败：{action_msg}")
        return

    if node.arm_status is None:
        print("❌ 失败：未收到 arm_status")
        return

    arm_code = node.arm_status.arm_status
    motion = node.arm_status.motion_status
    mode = node.arm_status.mode_feedback
    print(
        f"状态: arm_status={arm_code} "
        f"({ARM_STATUS_TEXT.get(arm_code, '未知')}), "
        f"motion_status={motion} "
        f"({'到达目标' if motion == 0 else '未到达'}), "
        f"mode_feedback={mode} ({MODE_TEXT.get(mode, '未知')})"
    )

    max_pos_err = max(abs(v) for v in pos_err)
    max_rpy_err = max(abs(v) for v in rpy_err)
    if arm_code != 0:
        print(
            f"❌ 失败：arm_status={arm_code} "
            f"({ARM_STATUS_TEXT.get(arm_code, '未知')})"
        )
    elif motion != 0:
        print(f"❌ 失败：未到达目标（motion_status={motion}）")
    elif max_pos_err > 2.0 or max_rpy_err > 0.5:
        print(
            f"⚠️ 状态成功但误差偏大：位置 {max_pos_err:.1f} mm，"
            f"姿态 {max_rpy_err:.2f}°"
        )
    else:
        print(
            f"✅ 成功：已到达目标（位置误差 {max_pos_err:.1f} mm，"
            f"姿态误差 {max_rpy_err:.2f}°）"
        )

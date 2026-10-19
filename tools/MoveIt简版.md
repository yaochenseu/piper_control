# MoveIt 简版

## 启动

终端 1（驱动，控制门先关闭）：

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
  can_port:=can0 arm_type:=piper_x effector_type:=agx_gripper \
  speed_percent:=15 control_enabled:=false
```

终端 2：

```bash
moveit                 # 默认开启门控
moveit use_rviz:=false # RViz 崩溃时用这个
```

## 控制

```bash
armplus j x x -30 x x x    # 控制器直连，不规划
armpro  j x x -30 x x x    # MoveIt 规划
armplus p x x x+10         # 笛卡尔 +10 mm
armpro  p x x x+10
armplus p x x x+10 -l      # 直线
```

- 关节/姿态：度；位置：毫米；夹爪：0~1
- `x` 保持当前，`x+10` 当前 +10，`x-10` 当前 -10
- `-n` 只预演
- 第 7 个参数是夹爪比例；`-f` 在 armplus/armpro 下不生效

## 注意

- MoveIt 运行时不要用 `arm`，用 `armplus` / `armpro`
- MoveIt 未启动时，armplus/armpro 会提示降级；`-y` 自动确认
- 速度由驱动 `speed_percent`（当前 15）限制；`armplus --speed`、`armpro --vel/--acc` 只能更慢

## arm / armplus / armpro 的区别

`arm` 不依赖 MoveIt，直接把指令发给驱动的 `/control/move_j`、`move_p`、`move_l`，速度最快，夹爪的 `-f` 参数也有效。但 MoveIt 运行时，ros2_control 会以 200 Hz 持续发布 `/control/joint_states`，覆盖 `arm` 的直接指令，表现为“发了不动”或抖动，所以 MoveIt 运行期间不要用 `arm`。

`armplus` 借 MoveIt 的 `/arm_controller/follow_joint_trajectory` 执行轨迹，本身不做规划和避障。p 模式先用 `/compute_ik` 求逆解再发关节轨迹，`-l` 用 `/compute_cartesian_path` 生成直线；夹爪走 `/gripper_controller`，`-f` 不生效；速度用 `--speed`（度/秒）控制。它适合 MoveIt 已启动、目标可以直达的普通点到点运动。

`armpro` 通过 `/move_action` 交给 move_group 规划，能避障，也能绕开关节限位。p 模式是逆解后再规划，`-l` 同样走 Cartesian 路径；速度用 `--vel` / `--acc` 缩放，代价是每次都要规划，最慢。它还要求 move_group 以真机反馈作为规划起点，也就是启动时必须 `follow:=true`（`moveit` 已默认带上）。

选择上，没启动 MoveIt 就用 `arm`；启动了 MoveIt、普通点到点用 `armplus`；需要规划/避障，或者 `armplus` 到不了目标，用 `armpro`。另外，MoveIt 没启动时，`armplus` / `armpro` 会询问是否降级为 `arm`；若检测到 MoveIt 控制器已在运行但能力不完整，则拒绝执行。

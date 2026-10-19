# MoveIt 控制说明

命令：

| 命令 | 作用 |
|---|---|
| `moveit` | 启动 MoveIt + RViz |
| `armplus` | MoveIt 控制器直连（Route A，不规划） |
| `armpro` | MoveIt 规划执行（Route B，/move_action） |

前置：驱动已在运行，且 `ROS_DOMAIN_ID` 一致（当前 42）。

`armplus` / `armpro` 的语法、单位和 `arm` 完全一致（度 / 毫米 / 夹爪比例）。

---

## 0 命令行控制：armplus / armpro

### 0.1 用法

```bash
armplus j x x -30 x x x            # 直连控制器
armplus j x+10 x x x x x 0.5
armplus p x x x+10                 # IK + 关节轨迹
armplus p 96.9 0 300 -85 0 -90 -l  # Cartesian 直线

armpro j x x -30 x x x             # MoveIt 规划
armpro p x x x+10                  # IK + MoveIt 规划
armpro j --vel 0.05 --planner RRTConnect x x -30 x x x
```

### 0.2 两者区别

| | `armplus` | `armpro` |
|---|---|---|
| 后端 | `/arm_controller/follow_joint_trajectory` | `/move_action`（move_group） |
| 规划/避障 | 无 | 有 |
| p 模式 | `/compute_ik` + 关节轨迹 | `/compute_ik` + 规划 |
| `-l` | `/compute_cartesian_path` | `/compute_cartesian_path` |
| 需要 `follow:=true` | 否 | **是** |
| 主要选项 | `--speed`（度/秒） | `--vel` `--acc` `--planner` `--plan-time` |

### 0.3 MoveIt 未启动时

- 如果 `/control/joint_states` **有发布者**（MoveIt 控制器已在运行）：
  拒绝执行，避免 `arm` 直连指令被 200 Hz 控制流覆盖。
  终端里可选 `r` 重试 / `q` 退出；非交互模式直接退出。
- 如果**没有**控制流：
  - 打印 `⚠️` 并询问是否降级为 `arm` 直连方式
  - 默认 `N`；`-y` 自动确认；`-n` 预演不询问
  - 非交互终端必须加 `-y`

`armpro` 还要求 `move_group` 订阅 `/feedback/joint_states`（`follow:=true`），
否则拒绝规划，避免用 mock 零位作为规划起点。

### 0.4 夹爪

`armplus` / `armpro` 都通过 MoveIt 的 `/gripper_controller` 控制夹爪：

- 第 7 个参数 0~1 → 0~100 mm
- **`-f` 力参数不生效**（控制器是位置接口），驱动使用默认约 1.0 N
- 需要精确夹持力时请用 `arm`

### 0.5 速度

- `armplus`：`--speed 20`（度/秒），默认 20
- `armpro`：`--vel 0.1` / `--acc 0.1`（MoveIt 缩放）
- 实际速度仍受驱动 `speed_percent=15` 限制
- 远距离运动建议先用 `armpro --vel 0.05` 试

### 0.6 重要安全提示：避免 MoveIt 启动时把机械臂拉回零位

`demo.launch.py` 的 ros2_control 使用 mock 硬件，初始位置来自
`initial_positions.yaml`（全零）。如果驱动以 `control_enabled:=true` 启动，
MoveIt 一启动就会通过 `/control/joint_states` 把真机拉到零位，**不管当时机械臂在哪**。

实机安全启动方式：

```bash
# 终端 1：驱动先关闭控制门
ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
  can_port:=can0 arm_type:=piper_x effector_type:=agx_gripper \
  speed_percent:=15 control_enabled:=false

# 终端 2：MoveIt 执行期自动门控
moveit auto_control_gate:=true
```

- MoveIt 空闲时不会向驱动发控制指令
- 只有执行 `arm_controller`、`gripper_controller` 或灵巧手 Action 轨迹时，
  门控节点才自动打开 `/control_enable`（已包含夹爪 Action）
- `armplus` / `armpro` 都走这些 Action，和门控兼容
- 这种模式下 `arm` 直连命令在 MoveIt 空闲时会被门控挡住；
  MoveIt 运行期间请用 `armplus` / `armpro`
- **Topic 方式** `/arm_controller/joint_trajectory` 不产生 Action 状态，
  门控不会开门；请改用 Action，或手动开门
- `moveit` 包装脚本默认 `auto_control_gate:=true`；
  显式传 `auto_control_gate:=false` 可关闭

---

## 1 启动

**实机推荐：门控启动**（避免 MoveIt 启动时把机械臂拉到零位）

```bash
# 终端 1：驱动，先关闭控制门
ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
  can_port:=can0 arm_type:=piper_x effector_type:=agx_gripper \
  speed_percent:=15 control_enabled:=false

# 终端 2：MoveIt，执行轨迹时自动开门
moveit            # 默认 auto_control_gate:=true
```

- `moveit` 自动带上 `follow:=true`，MoveIt 用真机反馈作为规划起点
- `moveit` 默认开启门控；需要关闭时显式写 `moveit auto_control_gate:=false`
- MoveIt 空闲时不会向驱动发控制指令
- RViz 崩溃时改用 `moveit use_rviz:=false`

> **顺序建议：先驱动，后 MoveIt。**
> 驱动先起，`move_group` 启动时就能拿到 `/feedback/joint_states`，
> 门控服务 `/control_enable` 也已就绪。
> 先起 MoveIt 也能运行，但在驱动起来之前无法规划，也看不到真实姿态。

一键启动（驱动未运行时，二者一起拉起）：

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \
  can_port:=can0 arm_type:=piper_x effector_type:=agx_gripper \
  speed_percent:=15 auto_control_gate:=true
```

> 一键启动会自动把 `control_enabled` 设为 `false`，但始终启动 RViz；
> 如果 RViz 不稳定，请用上面的两终端方式。

---

## 2 RViz 里控制

窗口左侧 **MotionPlanning** 面板。

### 2.1 关节/位姿规划（arm）

1. **Planning Group** 选 `arm`
2. **Start State** 保持 `<current>`
3. **Goal State** 可选预设 `home`（零位）
4. 或直接拖动末端彩色交互标记（6D 球）设定目标
5. 点 **Plan** 预览轨迹，点 **Execute** 下发真机
6. **Plan & Execute** = 规划并执行

### 2.2 关节滑条

**Joints** 标签页 → 拖动 `joint1`~`joint6`（单位弧度）→ **Plan & Execute**。

### 2.3 夹爪

1. **Planning Group** 切到 `gripper`
2. **Goal State** 选 `gripper_open` / `gripper_half` / `gripper_close`
3. **Plan & Execute**

夹爪关节名 `gripper`，单位米：0 = 闭合，0.1 = 全开。

### 2.4 速度

- **Planning** 标签页的 **Velocity Scaling / Acceleration Scaling** 滑条，
  默认 `0.1`（配置见 `joint_limits.yaml`）
- 实际速度还受驱动 `speed_percent`（当前 15%）限制，取两者较小值
- 首次实操建议把 Velocity Scaling 调到 `0.05`

---

## 3 控制链路

```text
RViz / MoveIt 规划
   ↓ /arm_controller/follow_joint_trajectory (Action)
JointTrajectoryController（ros2_control，200 Hz 插值）
   ↓ /control/joint_states
agx_arm_ctrl 驱动
   ↓ CAN
真机（Piper X + 夹爪）
```

MoveIt 侧的 ros2_control 使用 mock 硬件，只负责轨迹插值；
真正下发到机械臂的是 `agx_arm_ctrl`。

---

## 4 命令行下发轨迹

无需 RViz，直接调控制器 Action（弧度）：

```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: ['joint1','joint2','joint3','joint4','joint5','joint6'],
    points: [
      {positions: [0, 0, 0, 0, 0, 0], time_from_start: {sec: 2}},
      {positions: [0.2, 0.3, -0.3, 0, 0, 0], time_from_start: {sec: 5}}
    ]
  }
}"
```

或 Topic 方式（无执行反馈）：

```bash
ros2 topic pub --once /arm_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['joint1','joint2','joint3','joint4','joint5','joint6'],
  points: [{
    positions: [0.0, 0.2, -0.2, 0.0, 0.0, 0.0],
    time_from_start: {sec: 3}
  }]
}"
```

夹爪轨迹：把 `joint_names` 换成 `['gripper']`，位置 0~0.1 米。

---

## 5 常见问题

| 现象 | 原因/处理 |
|---|---|
| Plan 成功但不动 | 驱动未使能 → `ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: true}"` |
| 驱动日志 `control gate is closed` | 启动时设了 `auto_control_gate:=true`，需开门控服务 |
| `arm_status=4` | 目标超出关节限位，换目标 |
| 执行超时/跟不上 | 降低 RViz 的 Velocity Scaling；驱动速度 15% |
| `Failed to load sensor: point_cloud_sensor` | 未装点云插件，不影响机械臂控制，可忽略 |
| `recognize_objects not available` | 未启动感知，不影响控制，可忽略 |

---

## 6 关闭

- 在 `moveit` 所在终端 `Ctrl+C`
- 只关 MoveIt、保留驱动：结束后驱动仍可继续用 `arm j/p` 控制

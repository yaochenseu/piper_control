# Piper X 机械臂控制

AgileX Piper X + 夹爪的 ROS 2 控制代码：命令行控制脚本、MoveIt 配置，以及 `agx_arm_ros` 驱动源码。

## 目录

- `tools/` — 控制脚本：`arm`（直连驱动）、`armplus`（MoveIt 控制器直连）、`armpro`（MoveIt 规划）、`arm_state`、MoveIt 启动、手眼标定脚本。
- `agx_arm_ros/` — AgileX 驱动、MoveIt 配置和 URDF 模型（已包含 `piper_x` 等型号的网格文件）。

## 环境要求

- Ubuntu 22.04 + ROS 2 Humble
- MoveIt、ros2_control、can-utils
- 系统 Python 3.10（不要用 conda）
- [pyAgxArm](https://github.com/agilexrobotics/pyAgxArm)

```bash
sudo apt update
sudo apt install -y ros-humble-moveit ros-humble-ros2-control \
  ros-humble-ros2-controllers python3-colcon-common-extensions can-utils

git clone https://github.com/agilexrobotics/pyAgxArm.git ~/third_party/pyAgxArm
cd ~/third_party/pyAgxArm
git checkout d90e234
/usr/bin/python3 -m pip install --user python-can scipy numpy
/usr/bin/python3 -m pip install --user .
```

## 安装

```bash
git clone https://github.com/yaochenseu/piper_control.git ~/Desktop/piper/piper_control
mkdir -p ~/agx_arm_ws/src
ln -s ~/Desktop/piper/piper_control/agx_arm_ros ~/agx_arm_ws/src/agx_arm_ros
ln -s ~/Desktop/piper/piper_control/tools ~/agx_arm_ws/tools

mkdir -p ~/.local/bin
for f in arm armplus armpro arm_state moveit calib aruco_detect; do
  ln -sf ~/agx_arm_ws/tools/$f.sh ~/.local/bin/$f
done

cd ~/agx_arm_ws
colcon build --symlink-install
```

`*.sh` 默认 `ROS_DOMAIN_ID=42`，并自动 source `/opt/ros/humble/setup.bash` 和 `~/agx_arm_ws/install/setup.bash`。如果工作区不在 `~/agx_arm_ws`，需要改脚本里的路径。

## CAN

```bash
sudo ip link set can0 down 2>/dev/null
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

## 启动驱动

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
  can_port:=can0 arm_type:=piper_x effector_type:=agx_gripper speed_percent:=15
```

## 常用命令

```bash
arm j 0 0 0 0 0 0              # 六个关节回零（度）
arm j x x -30 x x x            # x = 保持当前值
arm j x+10 x x x x x 0.5       # 当前 +10°，第 7 个参数是夹爪 0~1
arm p x x x+10                 # 末端 +10 mm
arm p 96.9 0 300 -85 0 -90 -l  # 直线运动
arm_state                      # 查看当前状态
```

- 关节、姿态：度；位置：毫米；夹爪：0~1
- `x+10` / `x-10`：当前值加减；`-n` 只预演，不发送
- 超出关节限位会直接拒绝；`-f` 夹持力参数只对 `arm` 有效

## MoveIt

```bash
# 终端 1：驱动，先关闭控制门
ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
  can_port:=can0 arm_type:=piper_x effector_type:=agx_gripper \
  speed_percent:=15 control_enabled:=false

# 终端 2：MoveIt + RViz
moveit

# 终端 3
armplus j x x -30 x x x   # 控制器直连，不规划
armpro  j x x -30 x x x   # MoveIt 规划，可避障
```

- MoveIt 运行时不要用 `arm`，用 `armplus` / `armpro`
- MoveIt 未运行时，`arm` 会自动打开驱动控制门
- 详细说明见 `tools/MoveIt简版.md` 和 `tools/MoveIt控制说明.md`

## 手眼标定（可选）

依赖 `ros-humble-realsense2-camera` 和 `python3-opencv`。流程：驱动 → RealSense → `aruco_detect` → `calib`，完整步骤见 `tools/手眼标定说明.md`，当前结果见 `tools/手眼标定结果.md`。

```bash
sudo apt install -y ros-humble-realsense2-camera python3-opencv
ros2 launch realsense2_camera rs_launch.py \
  rgb_camera.color_profile:=1280x720x30 enable_depth:=false
aruco_detect --board charuco --dictionary DICT_4X4_50 \
  --squares-x 7 --squares-y 5 \
  --square-length 0.03 --marker-length 0.022 --blur 3
calib
```

标定板文件在 `tools/calib_board/`（ChArUco 7×5，方格 30 mm，码 22 mm）。标定不依赖 `aruco_ros` / `handeye_calibration_ros`。

## 说明

- 本仓库的 `agx_arm_ros` 来自 [AgileX 官方仓库](https://github.com/agilexrobotics/agx_arm_ros)，包含以下本地修改：驱动默认速度 15%、控制门覆盖夹爪/灵巧手 Action、RViz 当前状态改用 TF 驱动显示。
- 单位约定：关节/姿态用度，笛卡尔位置用毫米，夹爪用 0~1 比例。

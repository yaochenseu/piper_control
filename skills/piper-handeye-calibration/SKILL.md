---
name: piper-handeye-calibration
description: Automatically perform eye-in-hand hand-eye calibration for an AgileX Piper X arm with an Intel RealSense camera and a ChArUco board. Use when the user asks to calibrate camera-to-flange extrinsics, redo hand-eye calibration, or produce T_flange_cam after the camera/end effector is remounted.
---

# Piper X 手眼标定 Skill

This directory is a portable, platform-neutral skill. Any agent that can read
files and run shell commands can follow it; no Claude-specific API is used.

## What this skill produces

- `T_flange_cam` (camera in flange/TCP frame) as a 4x4 matrix, quaternion, RPY,
  and `<origin xyz rpy>` URDF snippet.
- A JSON result in `tools/calib_results/<timestamp>_handeye.json`.
- Residual metrics: translation RMS/max, rotation RMS/max, pose diversity.

## When to use

Use for **eye-in-hand** calibration: camera rigidly mounted on the arm, board
fixed in the workspace. Do not use for eye-to-hand; that requires inverting the
hand-eye equation and a different validation.

## Non-negotiable safety rules

1. **Never calibrate in teach mode.** `ctrl_mode` must be `1` (CAN control).
   `ctrl_mode: 2` means teaching mode; motion commands are not reliable.
2. **Never call `/exit_teach_mode` to leave teach mode.** The driver's
   implementation first moves all joints to zero, then does an emergency stop,
   reset, re-enable, and another zero move. It will yank the arm away from the
   board. Exit teach mode with the physical button/pendant instead.
3. Do not run MoveIt during calibration. Use the direct driver (`arm` / the
   skill scripts), not `armplus` / `armpro`.
4. Keep the workspace clear, the emergency stop reachable, and the board fixed.
5. The default speed is 5%. Do not raise it for calibration.
6. Print the ChArUco PDF at **100% / actual size**. Never "fit to page".

## Prerequisites

- Ubuntu 22.04 + ROS 2 Humble.
- Built workspace: `~/agx_arm_ws` containing `agx_arm_ctrl`,
  `agx_arm_description`, `agx_arm_msgs`, and `pyAgxArm`.
- `can0` at 1 Mbit/s.
- Intel RealSense D435i and `realsense2_camera`.
- System Python 3.10, OpenCV, SciPy, NumPy. Do not use conda.
- ChArUco board: 7x5, 30 mm squares, 22 mm ArUco markers,
  `DICT_4X4_50`. The printable PDF is in `assets/`.

Check CAN:

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details link show can0 | grep -E 'can0|bitrate|state'
```

Expected: `state UP`, `bitrate 1000000`. `ERROR-ACTIVE` is normal.

## Start the stack

Run each command in its own terminal (or in the background with logs). All
scripts honour `ROS_DOMAIN_ID` (default `42`), `AGX_WS` (default
`~/agx_arm_ws`), and `CAN_PORT` (default `can0`).

Terminal 1 — driver (auto-enable, control gate closed):

```bash
bash skills/piper-handeye-calibration/scripts/start_driver.sh
```

Wait for:

```text
All joints enable status is True
AgxGripper initialized successfully
```

Terminal 2 — RealSense color stream:

```bash
bash skills/piper-handeye-calibration/scripts/start_camera.sh
```

Terminal 3 — ChArUco detector:

```bash
bash skills/piper-handeye-calibration/scripts/start_detector.sh
```

Terminal 4 — preflight (no motion):

```bash
bash skills/piper-handeye-calibration/scripts/preflight.sh
```

The preflight checks CAN, services, feedback topics, camera, detector,
`ctrl_mode == 1`, `arm_status == 0`, and `err_status == 0`. It must print
`✅ 预检通过`.

## Position the arm before automatic collection

The auto script assumes the **whole board is visible** at 0.25–0.75 m and the
arm is already in a good baseline pose. Move the arm with your platform's
motion API, the vendor pendant, or the repo's `arm` command. A useful baseline
is the tip about 150–250 mm above the board center, with the camera looking at
the board.

Check visibility:

```bash
ros2 topic echo /aruco_single/pose --once
ros2 topic hz /aruco_single/pose
```

If no pose is published, fix visibility before continuing. Do not run the
auto script and hope it recovers from an invisible board.

## Dry run, then calibrate

Always inspect candidate poses first:

```bash
bash skills/piper-handeye-calibration/scripts/run_calibration.sh --dry-run
```

Then run the automatic collection:

```bash
bash skills/piper-handeye-calibration/scripts/run_calibration.sh \
  --samples 22 --rot-samples 12
```

The script:

1. Re-checks CAN control mode, feedback, camera intrinsics, and board
   visibility.
2. Opens `/control_enable`.
3. Applies small joint offsets from `config/default.json`, skipping poses that
   violate joint limits or lose the board.
4. Adds pure translations to improve position diversity.
5. Returns to the baseline pose.
6. Compares five OpenCV hand-eye algorithms, picks the lowest consistency
   score, and writes a JSON result.

Do not touch the arm, camera, or board while it is moving.

## Acceptance criteria

Accept only if all are true:

- translation RMS <= 3.0 mm (max should be reported too)
- rotation RMS <= 1.0 deg
- pose span >= 20 deg
- end-effector position spread >= 80 mm
- `|t| <= 0.5 m`

The script prints `✅ 标定通过验收标准` or `⚠️ 标定未通过验收标准`. If it
fails, inspect first; do not add an ad-hoc compensation offset.

## Result and use

The JSON contains:

```json
{
  "result": {
    "frame": "T_tcp_cam: camera in flange/TCP frame",
    "matrix4x4": [[...]],
    "position_mm": [...],
    "rpy_deg": [...],
    "translation_rms_mm": ...,
    "rotation_rms_deg": ...
  }
}
```

With driver `tcp_offset=0`, the TCP is the flange. To use the result, update the
camera mount transform in the URDF/TF tree or the perception stack. Do not
change the robot's joint zero offsets to compensate for a hand-eye residual.

## Common failures

| Symptom | Action |
|---|---|
| `ctrl_mode: 2` | Exit teach mode with the physical button/pendant; never via `/exit_teach_mode`. |
| `没有 /aruco_single/pose` | Check camera image, board in view, detector args, same `ROS_DOMAIN_ID`. |
| Board pose unstable | Fix the board; avoid hard paper flex; wait for the arm to stop. |
| Many candidates skipped | Baseline is near a joint limit; move to a more central visible pose. |
| Translation RMS > 3 mm | Check board flatness/size, lighting, motion blur, and that samples were taken while stationary. |
| Pose span < 20 deg | Increase wrist rotations; do not rely only on translations. |
| Result `\|t\| > 0.5 m` | Samples are degenerate or the board/camera moved; discard and retry. |

## Files

- `scripts/auto_calibrate.py` — automatic collector and solver.
- `scripts/calib.py` — hand-eye math, averaging, validation, JSON writer.
- `scripts/aruco_detect.py` — ChArUco detector publishing `/aruco_single/pose`.
- `scripts/start_*.sh` — driver, camera, detector.
- `scripts/preflight.sh` — non-moving checks.
- `scripts/run_calibration.sh` — entry point.
- `config/default.json` — board, topics, limits, candidates, result path.
- `assets/charuco_A4_7x5_30mm_22mm.pdf` — board to print at 100%.

The `calib.py` and `aruco_detect.py` copies are vendored from `tools/` so the
skill can be used when the skill directory is copied on its own. If the repo
tools change, re-copy them.

## Agent contract

- Read this file completely before acting.
- Run `preflight.sh`; stop on any `❌`.
- Run `run_calibration.sh --dry-run`; inspect for skipped/unsafe poses.
- Run the real calibration only when the board is visible and the workspace is
  clear.
- Report the result JSON path, T_flange_cam, and residuals.
- On failure, report the exact failing stage; do not invent coordinates or add
  compensation.
- For a different robot, edit `config/default.json` joint limits, candidates,
  board geometry, and topics first.

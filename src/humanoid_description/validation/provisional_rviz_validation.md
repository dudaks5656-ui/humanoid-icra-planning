# Provisional Humanoid RViz Validation

## Status and scope

- Validation date: 2026-08-11 (Asia/Seoul)
- Operating system: Ubuntu 22.04
- ROS distribution: ROS 2 Humble
- ROS domain: `ROS_DOMAIN_ID=42`
- Network isolation: `ROS_LOCALHOST_ONLY=1`
- Model status: `PROVISIONAL_VISUALIZATION_ONLY`
- Reused model content: OpenArm v1.0 left/right seven-axis arms and parallel-link grippers only
- Excluded content: OpenArm body, `openarm_robot`, ros2_control, controller manager, hardware plugins, MoveIt, and physical devices

This validation confirms only the displayed kinematic connection and joint-direction candidates. It is not paper data, final simulation data, a hardware configuration, or evidence of collision accuracy.

## User-confirmed RViz observations

The user directly confirmed the following in RViz:

- Increasing the positive value of `lift_joint` moves the upper body downward.
- `waist_yaw_joint` rotates about the vertical axis.
- `waist_pitch_joint` rotates about a horizontal axis.
- Both OpenArm assemblies move together with `waist_pitch_link`.
- The user visually checked the left/right arms for inversion and origin overlap.

Joint State Publisher GUI displayed `lift_joint`, `waist_yaw_joint`, `waist_pitch_joint`, both seven-axis arms, and both grippers.

## Static validation summary

- Xacro expansion: passed
- XML validation (`xmllint`): passed
- URDF validation (`check_urdf`): passed
- Root link: `world`
- Link count: 29
- Joint count: 28
- Fixed joints: 7
- Revolute joints: 16
- Prismatic joints: 5
- Duplicate links/joints: none
- Missing parent/child references: none
- Missing OpenArm mesh URIs: none
- `ros2_control` elements: 0
- `hardware` elements: 0
- `plugin` elements: 0
- `transmission` elements: 0
- OpenArm body links: 0

The only RViz errors were the pre-existing unrealistic-inertia-box messages for the four OpenArm finger links. They do not indicate a mesh or TF loading failure.

## Provisional geometry and joint placement

Every numeric value in this section is `PROVISIONAL_VISUALIZATION_ONLY`. None may be used for a paper, final collision analysis, final simulation, controller configuration, or physical hardware.

| Item | Current provisional value | Required replacement |
|---|---|---|
| AMR primitive size | box `0.8 0.6 0.2` m | Measured AMR visual/collision envelope |
| AMR primitive origin | xyz `0 0 0.1`, rpy `0 0 0` | Measured geometry origin |
| Lift joint origin | xyz `0 0 0.2`, rpy `0 0 0` | Measured `amr_base_link` to `CS_LIFT_TOP_ZERO` transform |
| Lift primitive | cylinder radius `0.06` m, length `0.50` m | CAD geometry or supplied STL |
| Lift primitive origin | xyz `0 0 0.25`, rpy `0 0 0` | Measured/CAD geometry origin |
| Yaw joint origin | xyz `0 0 0.50`, rpy `0 0 0` in `lift_link` | Measured Lift-to-Yaw transform |
| Yaw primitive | cylinder radius `0.12` m, length `0.12` m | CAD geometry or supplied STL |
| Yaw primitive origin | xyz `0 0 0.06`, rpy `0 0 0` | Measured/CAD geometry origin |
| Pitch joint origin | xyz `0 0 0.12`, rpy `0 0 0` in `waist_yaw_link` | Measured Yaw-to-Pitch transform |
| Pitch primitive | box `0.18 0.50 0.18` m | CAD geometry or supplied STL |
| Pitch primitive origin | xyz `0 0 0.09`, rpy `0 0 0` | Measured/CAD geometry origin |
| Left arm mount pose | xyz `0 0.30 0.09`, rpy `-1.570796 0 0` | Inventor `waist_pitch_link` to left mount transform |
| Right arm mount pose | xyz `0 -0.30 0.09`, rpy `1.570796 0 0` | Inventor `waist_pitch_link` to right mount transform |

The mount frames are currently defined to coincide with `openarm_left_link0` and `openarm_right_link0`; therefore each mount-to-link0 fixed transform is zero. CAD must confirm this frame definition.

## Provisional joint axes and limits

All limit, effort, and velocity values remain `PROVISIONAL_VISUALIZATION_ONLY` and are forbidden for paper, final simulation, control, or hardware use.

| Joint | Axis | Lower | Upper | Effort | Velocity | Status |
|---|---|---:|---:|---:|---:|---|
| `lift_joint` | `0 0 -1` | `0.0` m | `0.10` m | `1.0` | `0.10` m/s | Top-zero/downward direction confirmed; maximum stroke, effort, and velocity unmeasured |
| `waist_yaw_joint` | `0 0 1` | `-0.785398` rad | `0.785398` rad | `1.0` | `0.50` rad/s | Vertical rotation visually confirmed; limits/dynamics unmeasured |
| `waist_pitch_joint` | `0 1 0` candidate | `-0.523599` rad | `0.523599` rad | `1.0` | `0.50` rad/s | Horizontal rotation visually confirmed; final axis sign, limits, and dynamics unmeasured |

Lift uses the confirmed convention:

- Coordinate-system name: `CS_LIFT_TOP_ZERO`
- `q=0`: highest Lift position
- Increasing q: downward motion
- Axis: `0 0 -1`
- Lower limit: `0`
- Upper limit: must be replaced by the measured maximum downward stroke

## Freeze condition

Until the required CAD/Inventor measurements are supplied:

- Do not change any coordinate, joint origin, primitive dimension, arm-mount pose, joint axis/sign, or joint limit.
- Do not generate or substitute STL files.
- Do not use the provisional model for paper experiments, final simulation, collision conclusions, controller design, or hardware operation.
- Preserve the current Xacro, expanded URDF, launch, YAML, and RViz configuration files.

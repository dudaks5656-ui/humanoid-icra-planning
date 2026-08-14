# Measured Joint Position Limits Validation

## Validation context

- Validation date: 2026-08-11 (Asia/Seoul)
- Operating system: Ubuntu 22.04
- ROS distribution: ROS 2 Humble
- ROS domain: `ROS_DOMAIN_ID=42`
- Network isolation: `ROS_LOCALHOST_ONLY=1`
- Validation method: Xacro expansion, XML/URDF static checks, Joint State Publisher GUI, and direct user observation in RViz
- Scope: measured position limits and displayed joint directions only

No ros2_control, controller manager, physical hardware, Serial, CAN, or USB device was used.

## Applied measured position limits

| Joint | Applied range | Axis | Confirmed direction |
|---|---|---|---|
| `lift_joint` | `0.0` to `0.700` m | `0 0 -1` | Positive motion moves downward from the top q=0 position |
| `waist_yaw_joint` | `-0.17453292519943295` to `0.17453292519943295` rad | `0 0 1` | Front-referenced left/right rotation is normal; -10° clockwise and +10° counterclockwise from above |
| `waist_pitch_joint` | `-0.17453292519943295` to `0.7853981633974483` rad | `0 1 0` candidate | -10° backward to +45° forward; positive motion was directly confirmed as forward tilt |

Human-readable ranges:

- Lift: `0` to `0.700 m`
- Yaw: `-10°` to `+10°`
- Pitch: backward `-10°` to forward `+45°`

## Direct RViz confirmation

The user directly confirmed:

- Positive `lift_joint` motion is downward.
- `waist_yaw_joint` rotates normally left/right about its vertical axis from the forward-facing zero pose.
- Positive `waist_pitch_joint` motion tilts the upper body forward.
- The current Lift/AMR intersection is caused by provisional primitive geometry and a provisional joint origin, not by the measured position-limit values.

## Values that remain provisional

The existing effort and velocity entries have not been measured. They remain marked `PROVISIONAL_NOT_MEASURED` and must not be used for control, hardware, final simulation, safety analysis, or paper results.

The following also remain provisional and unchanged:

- Lift, Yaw, and Pitch joint origins
- Left and right arm-mount transforms
- AMR, Lift, Yaw, and Pitch primitive dimensions and origins
- Mesh selection for all non-OpenArm modules
- CAD/Inventor UCS coordinates and frame transforms

The AMR intersection must be resolved only after the actual CAD UCS and mesh/geometry data are supplied. It must not be corrected by guessing a new origin or primitive dimension.

## Static validation result

- Xacro expansion: passed
- XML validation (`xmllint`): passed
- URDF validation (`check_urdf`): passed
- Root link: `world`
- Links: 29
- Joints: 28
- Duplicate links/joints: none
- Missing parent/child references: none
- `ros2_control`, `hardware`, `plugin`, and `transmission` elements: 0
- All three applied lower/upper/axis checks: exact match

## SHA-256 of validated inputs and expanded URDF

```text
3bbf02c8d9904a28ec0d3e4ec607ee4ce738707c43c1502c1fbe67183cc0dd1b  /home/openarm/humanoid_sim_ws/src/humanoid_description/config/measured_joint_limits.yaml
317341a3ec853c851879b71071f6dcbc47411d10c65c373cb086e22cdaa91281  /home/openarm/humanoid_sim_ws/src/humanoid_description/config/provisional_limits.yaml
db7ab3ac11b2432514a0ee9d33a3d44f24edf9190368c37df7fbc3746fef55ec  /home/openarm/humanoid_sim_ws/src/humanoid_description/config/provisional_geometry.yaml
3035c338786dea727ac6d22bb5c5fec8ac7e5cf8981258d41665e4ee4f176d85  /home/openarm/humanoid_sim_ws/src/humanoid_description/validation/humanoid_expanded.urdf
```

The SHA-256 of this validation document is stored separately in `measured_joint_limits_validation.sha256` to avoid a self-referential checksum.

## Freeze condition

Until the CAD UCS and mesh data are supplied, do not change joint coordinates, joint origins, arm mounts, primitive geometry, mesh references, or joint limits. Do not generate STL files or use this partially provisional geometry for final experiments.

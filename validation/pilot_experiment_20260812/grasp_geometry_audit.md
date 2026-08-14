# Grasp geometry audit

Date: 2026-08-12 (Asia/Seoul)

All values below are simulation geometry measurements. No robot Xacro/URDF, SRDF/ACM, joint limit, joint origin, collision mesh, kinematics, or OMPL setting was changed.

## Source geometry

- TCP: `openarm_left_hand_tcp`
- TCP transform from `openarm_left_link7`: fixed, `xyz=0 0 0`, `rpy=0 0 0`
- Finger collision mesh: `package://openarm_description/assets/end_effector/parallel_link/meshes/collision/finger.stl`
- Mesh triangles: 264
- Raw STL AABB [mm]: min `[-30.465065, 42.238811, 658.500488]`, max `[30.500973, 77.061432, 753.421265]`
- Left finger joint transform at q: `[0, +q, 0.1025] m`
- Right finger joint transform at q: `[0, -q, 0.1025] m`
- Left collision origin: `[0, -0.05, -0.673001] m`, scale `[0.001, 0.001, 0.001]`
- Right collision origin: `[0, +0.05, -0.673001] m`, scale `[0.001, -0.001, 0.001]`

The independent and mimic axes are opposite, so increasing q increases the separation by `2q`. The URDF upper limit is exactly `0.044 m`; this value is used only as a simulation planning aperture and is not claimed as a hardware-validated fully-open position.

## TCP-frame AABB at q_open = 0.044 m

| Link | AABB min [m] | AABB max [m] | AABB center [m] |
|---|---|---|---|
| `openarm_left_left_finger` | `[-0.030465065, 0.036238811, 0.087999488]` | `[0.030500973, 0.071061432, 0.182920265]` | `[0.000017954, 0.053650122, 0.135459876]` |
| `openarm_left_right_finger` | `[-0.030465065, -0.071061432, 0.087999488]` | `[0.030500973, -0.036238811, 0.182920265]` | `[0.000017954, -0.053650122, 0.135459876]` |

- Inner Y gap at q=0: `-0.015522377 m` (AABB overlap)
- Inner Y gap at q=0.0066: `-0.002322377 m` (AABB overlap)
- Inner Y gap at q=0.0088: `+0.002077623 m`
- Inner Y gap at q=0.044: `+0.072477623 m`
- Current target Y width: `0.050 m`
- Nominal side clearance at q=0.044: `(0.072477623 - 0.050)/2 = 0.011238812 m` per side

The midpoint of the two collision-AABB centers is `[0.000017954, 0, 0.135459876] m` in the TCP frame. Projected onto the verified local +Z approach axis:

```text
tcp_to_grasp_center = 0.13545987646484376 m
```

The negligible X component is caused by STL AABB asymmetry and is not used as an invented robot-frame correction.

## Stage distances

- `tcp_to_grasp_center`: `0.13545987646484376 m`, measured from collision geometry
- `pre_grasp_clearance`: selected by FCL scan
- `insertion_offset`: `0.02 m`, explicitly marked `PROVISIONAL_DEVELOPMENT_VALUE`
- `lift_distance`: `0.08 m`, unchanged provisional task value
- `extraction_distance`: `0.24 m`, unchanged provisional task value

FCL scanned pre-grasp clearance from 0.05 through 0.30 m in 0.01 m steps at q=0.044. The first tested value, 0.05 m, had valid IK and no self, target, or box collision. Adding the required 0.01 m margin selected:

```text
pre_grasp_clearance = 0.06 m
```

The same orientation is used at all five stages: RPY `[0, pi/2, 0]`, quaternion xyzw `[0, 0.7071067811865475, 0, 0.7071067811865476]`.

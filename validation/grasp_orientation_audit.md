# Grasp orientation audit

Date: 2026-08-12 (Asia/Seoul)

This audit changes only the provisional experiment goal orientation. Robot Xacro/URDF, SRDF/ACM, joint limits, collision meshes, kinematics, OMPL configuration, controllers, and hardware files were not changed.

## Frames and approach geometry

- Planning group: `left_arm`
- End-effector/TCP link: `openarm_left_hand_tcp`
- `openarm_left_link7 -> openarm_left_hand_tcp`: fixed joint, `xyz=0 0 0`, `rpy=0 0 0`. The TCP local axes are therefore exactly the `openarm_left_link7` axes.
- Finger bases: both are attached at `xyz=0 0 0.1025` in `openarm_left_link7`.
- Independent finger axis: local `0 -1 0`; mimic finger axis: local `0 1 0`. The grasp closing direction is therefore local `±Y`.
- Box center: `[0.70, 0.20, 1.12] m`; depth is along world X.
- The back wall is at positive X and there is no front wall at negative X. The outside-to-inside insertion vector is world `+X`.
- Confirmed physical approach axis from the URDF gripper chain: end-effector local `+Z`.

The geometry-first orientation maps local `+Z -> world +X` and local `+Y -> world +Y`. Its rotation matrix, with local axes as columns in world coordinates, is:

```text
[ 0  0  1 ]
[ 0  1  0 ]
[-1  0  0 ]
```

## Roll candidates about the aligned approach axis

All four candidates have 0 deg approach-axis alignment error.

| Roll | Quaternion xyzw | RPY rad | Local closing axis in world | Geometric assessment | Default-state APPROACH IK | Collision result |
|---:|---|---|---|---|---|---|
| 0 deg | `[0, 0.7071067811865475, 0, 0.7071067811865476]` | `[0, 1.5707963267948966, 0]` | local +Y -> world +Y | Aligns with the target's two Y-normal side faces | Success | Collision: both finger links vs `target_object` |
| +90 deg | `[0.5, 0.5, 0.5, 0.5]` | `[1.5707963267948966, 0, 1.5707963267948966]` | local +Y -> world +Z | Closes vertically toward top/bottom; not preferred | Success | Collision: both finger links vs `target_object` |
| -90 deg | `[-0.5, 0.5, -0.5, 0.5]` | `[-1.5707963267948966, 0, -1.5707963267948966]` | local +Y -> world -Z | Closes vertically toward bottom/top; not preferred | Success | Collision: both finger links vs `target_object` |
| 180 deg | `[0.7071067811865475, 0, 0.7071067811865475, 0]` | `[3.141592653589793, -1.5707963267948966, 0]` | local +Y -> world -Y | Side-face alignment equivalent to 0 deg, with finger identities swapped | Success | Collision: both finger links vs `target_object` |

The 0 deg roll was selected on grasp-face geometry, not planning success. It preserves local +Y -> world +Y and uses a normalized quaternion with positive `w`. The exact same quaternion is generated for APPROACH, INSERTION, GRASP_POSE, LIFT, and EXTRACTION; only position changes, so no quaternion sign flip or artificial rotation is introduced.

## Applied orientation and backup

- Target position retained: `[0.50, 0.30, 1.25] m`
- Selected RPY: `[0, 1.5707963267948966, 0] rad`
- Selected quaternion xyzw: `[0, 0.7071067811865475, 0, 0.7071067811865476]`
- Before backup: `/home/openarm/humanoid_sim_ws/validation/backups/grasp_orientation_20260812_052149/confined_scene_before_orientation.yaml`
- Before SHA-256: `9d8277642ce91a5ce9da53c7a6678cb688cf9dfc011331a5a740d2f2fbbd2a97`
- Applied YAML SHA-256: `f2dabc7c65486c0d1d4116cf2af8a8c9aaa901a70fde42aec9df073f11435db9`

## Three-repeat result

Three identical planning-only runs were performed. In all three runs:

- `TORSO_LOCKED` failed at `APPROACH`.
- IK succeeded.
- FCL rejected the IK state because `openarm_left_left_finger` and `openarm_left_right_finger` collided with `target_object`.
- `TORSO_CANDIDATE_SEARCH` examined 25 in-limit Yaw/Pitch candidates and excluded five 45 deg Pitch candidates outside the exact URDF limit without clamping.
- No candidate completed APPROACH through EXTRACTION.
- Failure occurred before any OMPL planning request for the affected state. Consequently, an OMPL seed was not consumed and planning-seed sensitivity is not measurable for this failure. The three repeated pre-planning results were identical.

At APPROACH, the target center is always `0.14 + 0.025 + 0.020 = 0.185 m` along the aligned TCP local +Z direction. The finger joint origins are already `0.1025 m` along local +Z and their collision meshes extend farther. Because both the target and TCP stage positions are translated together, changing target XYZ alone does not change this relative 0.185 m separation and cannot remove this particular contact.

## Decision and safety

The orientation is geometrically correct, but the provisional pre-grasp/object envelope is inconsistent with the detailed gripper collision geometry. A further target-position search is not justified for this failure. A later step requires explicit approval to change a task/scene development value such as pre-grasp distance or target size, or to redefine how the non-attached target is handled during pre-grasp. No robot-model or ACM change is recommended.

No trajectory was executed. RViz was not opened because no successful recovery trajectory exists. No controller, ros2_control process, hardware interface, or real robot node was used.

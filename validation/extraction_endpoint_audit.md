# Extraction endpoint audit

Generated: 2026-08-12 06:31 KST

## Scope and invariants

- Robot Xacro/URDF, SRDF/global ACM, joint limits/origins, collision meshes, kinematics, and OMPL settings were not changed.
- Orientation remains RPY `[0, pi/2, 0]`.
- `tcp_to_grasp_center=0.13545987646484376 m`, `insertion_offset=0.02 m`, `q_open=0.044 m`, and target size `[0.05, 0.05, 0.08] m` remain unchanged.
- No trajectory was executed; no controller, ros2_control, or hardware node was used.

## Box and direction audit

- Box center X: `0.700 m`
- Interior depth: `0.450 m`
- Open front interior plane: `0.700 - 0.450/2 = 0.475 m`
- Wall thickness: `0.025 m`
- Inside direction: world `+X`
- Outside direction: world `-X`
- Target extent along approach axis: `0.050 m` (half extent `0.025 m`)
- Legacy fixed extraction distance: `0.240 m` (`DEPRECATED_NOT_USED`)

The old implementation computed `extraction_tcp = lift_tcp - world_X * extraction_distance`, producing the previously reported TCP X near `0.124540124 m`. This is deprecated because translating the initial target also translated the final endpoint and changed its physical meaning.

## Attachment transform and corrected equation

At nominal GRASP, the target is `0.13545987646484376 m` along TCP local `+Z`. The nominal TCP-to-target transform has translation `[0, 0, 0.13545987646484376] m` and rotation `Ry(-pi/2)`. At runtime the implementation stores the actual transform from the planned grasp state:

`T_tcp_target = inverse(T_world_tcp_at_grasp) * T_world_target_at_grasp`

It then defines the final target pose first and recovers the TCP pose as:

`T_world_tcp_goal = T_world_target_goal * inverse(T_tcp_target)`

Because outside is `-X`, the correct target-center equation is:

`object_center_x = box_front_x - object_half_extent_x - extraction_clearance`

This sign was verified against the box frame; the object's box-side (`+X`) face is therefore clearance metres outside the front plane.

## Clearance results

| Clearance (m) | Final object center X (m) | Nominal final TCP X (m) | Box-side face X (m) | Fully outside | Extraction IK exists |
|---:|---:|---:|---:|:---:|:---:|
| 0.005 | 0.445 | 0.309540123535156 | 0.470 | yes | yes |
| 0.010 | 0.440 | 0.304540123535156 | 0.465 | yes | yes |
| 0.020 | 0.430 | 0.294540123535156 | 0.455 | yes | yes |
| 0.030 | 0.420 | 0.284540123535156 | 0.445 | yes | yes |

The smallest qualifying value, `0.005 m`, was selected. The full per-candidate IK and collision audit is in `extraction_clearance_search.csv`. Since an endpoint was reachable at the original box placement, scene translation was not authorized or performed.

## Planning observations

At the original target `[0.50, 0.30, 1.25] m`, corrected endpoint planning was repeated three times with planning time `2 s` and `2` attempts per stage:

- attempt_1: LOCKED success; candidate search success at yaw/pitch `0/0`.
- attempt_2: LOCKED failed at APPROACH due stochastic `MOTION_PLANNING_FAILURE`; the identical `0/0` candidate succeeded, so this is not torso recovery.
- attempt_3: LOCKED failed at GRASP; yaw `-5 deg`, pitch `0 deg` completed all five stages. Because LOCKED succeeded in attempt_1, this is not a repeatably structural LOCKED failure.

Following the previously approved target-position-only search, the target was moved only toward the `+Y` interior corner to `[0.50, 0.35, 1.25] m`; it was not moved in `+X`, and the box was not translated. In that trial both modes failed. TORSO_CANDIDATE_SEARCH most commonly first failed at INSERTION with `box_left_wall | openarm_left_left_finger`; other candidates failed IK or had the self-collision pairs recorded in `single_case_extraction.csv`.

No further target or scene search was performed after the both-modes-failed stop condition.

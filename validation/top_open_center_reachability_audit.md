# Top-open center reachability audit

- Scene: `TOP_OPEN_BOX_600X400X150_GEOMETRY_VALIDATION`
- Scope: explicit-seed IK and static RobotState collision checks only
- IK solver: `kdl_kinematics_plugin/KDLKinematicsPlugin` for `left_arm`
- IK seeds per valid candidate/stage: 30
- Per-seed IK timeout: 0.01 s
- Candidates excluded by exact URDF limits: 75
- MoveGroup/OMPL/trajectory/attach/execution/controller/hardware: not used

## Stage poses

- APPROACH TCP: [0.675, 0.2, 1.23546], quaternion xyzw [0, 1, 0, 0]
- GRASP TCP: [0.675, 0.2, 1.10046], quaternion xyzw [0, 1, 0, 0]
- PRE_GRASP TCP: [0.675, 0.2, 1.13546], quaternion xyzw [0, 1, 0, 0]

## Stage feasibility across candidates

| Stage | LIFT_ONLY | LIFT_YAW_PITCH | Proposed-only |
|---|---|---|---|
|APPROACH|possible|possible|no|
|PRE_GRASP|possible|possible|no|
|GRASP|not found|not found|no|

## Best complete candidates by minimum collision clearance

- LIFT_ONLY: no candidate made all three stages collision-free
- LIFT_YAW_PITCH: no candidate made all three stages collision-free

Observed failed-solution collision pairs: `base_link<->openarm_right_left_finger;base_link<->openarm_right_right_finger;box_floor<->openarm_left_left_finger;box_floor<->openarm_left_right_finger;box_front_wall<->openarm_left_link3;box_front_wall<->openarm_left_link4;lift_fixed_link<->openarm_right_left_finger;lift_fixed_link<->openarm_right_link5;lift_fixed_link<->openarm_right_link6;lift_fixed_link<->openarm_right_link7;lift_fixed_link<->openarm_right_right_finger`

## Gate for reference generation

LIFT_ONLY did not make all three stages feasible. Do not generate an OMPL reference path.

## Detailed post-run aggregation

- CSV data rows: `35,325` plus one header
- Valid candidates: `15` LIFT_ONLY and `375` LIFT_YAW_PITCH
- Excluded candidates: `75` exact 45-degree Pitch candidates, because mathematical `pi/4` exceeds the current URDF upper literal `0.785398 rad` by about `1.63e-7 rad`; no clamping was applied.

### LIFT_ONLY

- APPROACH: `89` IK / `89` collision-free solutions; feasible Lift values `0.25, 0.30, 0.35 m`
- PRE_GRASP: `88` IK / `88` collision-free solutions; feasible Lift values `0.35, 0.40, 0.45 m`
- GRASP: `120` IK / `0` collision-free solutions; Lift values with IK `0.35, 0.40, 0.45, 0.50 m`
- Lift `0.35 m` is the only Lift value common to collision-free APPROACH and PRE_GRASP. Its best environment clearances were about `0.063748 m` and `0.011239 m`, respectively.
- There is no valid "most-clearance Lift" for the complete three-stage task because every GRASP state failed.

### LIFT_YAW_PITCH

- APPROACH: `3,932` IK / `2,252` collision-free solutions across `77` torso candidates
- PRE_GRASP: `3,922` IK / `2,063` collision-free solutions across `76` torso candidates
- GRASP: `3,994` IK / `0` collision-free solutions
- No stage is feasible only with Yaw/Pitch: APPROACH and PRE_GRASP already work in LIFT_ONLY, while GRASP fails in both modes.

### Exact GRASP failure

All `4,114` GRASP IK solutions (`120 + 3,994`) contained both invariant contacts:

- `box_floor <-> openarm_left_left_finger`
- `box_floor <-> openarm_left_right_finger`

Some solutions additionally contacted `box_front_wall` with `openarm_left_link3` or `openarm_left_link4`, and some nonzero-torso states introduced right-arm/lift self-collisions. The invariant two-finger/floor contact is sufficient to reject every GRASP state.

This is not an IK failure and cannot be repaired merely by enabling Yaw/Pitch. With the grasp center aligned to the 50 mm cube center, the existing finger collision geometry extends below the floor plane in the approved top-entry orientation.

## Final gate decision

APPROACH and PRE_GRASP are feasible in LIFT_ONLY, but GRASP is not. The required three-stage LIFT_ONLY gate is therefore **not satisfied**, and no OMPL reference trajectory may be generated from this result.

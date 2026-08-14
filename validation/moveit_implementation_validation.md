# MoveIt implementation validation

Date: 2026-08-12 (Asia/Seoul)

## Scope and safety

- Robot description: existing `humanoid_sim.urdf.xacro`, unchanged.
- URDF root: `world`; existing `world_to_base_joint` remains fixed.
- SRDF virtual joint: none.
- Collision detector: Humble default FCL; Bullet was not selected.
- Trajectory execution: disabled (`allow_trajectory_execution=false`).
- ExecuteTrajectory capability: disabled.
- Controllers and hardware drivers: not launched.

## Runtime configuration

- Planning pipeline: `ompl_interface/OMPLPlanner`.
- Planner for every planning group: `RRTConnectkConfigDefault`.
- Baseline `longest_valid_segment_fraction`: `0.01` (runtime queried).
- Fine comparison fraction: `0.0025` (`PROVISIONAL_VALIDATION_SETTING`).
- `maximum_waypoint_distance`: not configured; unsupported in the audited Humble interface.
- Trajectory revalidation subdivisions: coarse 1, medium 5, fine 20; provisional comparison settings.

## Full-state and SCM validation

- Independent RobotState variables: 19.
- Random full-state SCM samples: 10,000 completed.
- Mean self-collision check time: 1.019 ms.
- SRDF disabled pairs: 22, all `Adjacent`.
- Protected important pairs audited: 196.
- Protected pairs incorrectly disabled: 0.
- Pairs sampled as never colliding were not disabled.
- Initial collisions were not disabled.

Initial all-zero state:

- Joint limits: valid.
- Self collision: true (3 pairs).
- Environment collision: false.
- Pairs:
  - `openarm_left_left_finger | openarm_left_right_finger`
  - `openarm_left_link0 | waist_pitch_link`
  - `openarm_right_left_finger | openarm_right_right_finger`

`openarm_left_link0 | waist_pitch_link` occurred in 10,000/10,000 random states.
It may represent fixed bolted contact at the profile, but it remains enabled
because Pitch structure versus arm collision was explicitly designated as an
important checked pair. Geometry/ACM was not changed to hide this collision.
Until this contact is classified from CAD/assembly intent, OMPL planning may
reject every state.

## Runtime warnings

- No 3D sensor plugin is configured; MoveIt logged an Octomap warning. No sensor or hardware was accessed.
- The controller list is deliberately empty; MoveIt reports zero controllers. Planning remains available, execution does not.
- On Ctrl-C, `robot_state_publisher` stopped cleanly. Humble `move_group` reproduced a class-loader teardown segmentation fault after its planning scene monitors stopped. No robot command was sent.
- The standalone collision checker stopped cleanly with Ctrl-C.

Detailed SCM output: `/home/openarm/humanoid_sim_ws/validation/self_collision_matrix_audit.txt`

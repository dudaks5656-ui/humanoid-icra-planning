# Front-open development reference classification

- Classification: `FRONT_OPEN_DEVELOPMENT_REFERENCE_NOT_FOR_FINAL_EXPERIMENT`
- Date preserved: 2026-08-12 (Asia/Seoul)
- Scene: existing front-open development box
- Target: `[0.50, 0.30, 1.25]` m
- Mode: `LIFT_ONLY`
- Selected torso state: Lift `0.10 m`, Yaw `0 rad`, Pitch `0 rad`
- Trajectory execution: **not performed**

## What was verified

- The full-task path-generation and target-object lifecycle pipeline operated end to end.
- Ten full-task planning trials produced five candidates that passed the recorded dense collision validation.
- Trial 7 was selected, with 103 source waypoints and 395 densely interpolated validation states.
- The selected result recorded no joint-limit violation, self-collision, development-box collision, attached-object/box collision, or inter-stage RobotState discontinuity.

## Why this is not a final reference

- The trajectory was generated in the **front-open development scene**, not the final top-open box experiment.
- `openarm_left_joint4` and `openarm_left_joint5` reached their exact joint boundaries; the active-joint margin is `0`.
- Maximum orientation error was `0.03778 rad`, exceeding the configured `0.03 rad` tolerance.
- Minimum self-collision clearance was only `0.00065926 m` (`0.65926 mm`), so this is not considered a robust path.
- It must not be used as a final reference trajectory or paper performance result.

## Preservation rule

These files are immutable development evidence. They must not be recalculated, edited, or mixed with any future reference generated for the top-open 600 x 400 x 150 mm scene.

## Session shutdown note

The launch session was stopped with SIGINT. RViz and `robot_state_publisher` exited cleanly. MoveGroup emitted the previously observed class-loader shutdown failure (`exit code -11`) after the completed result files had already been written; this shutdown-only event does not reclassify the preserved planning result.

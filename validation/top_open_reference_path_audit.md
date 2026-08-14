# Top-open offline reference path audit

- Scene: TOP_OPEN_BOX_600X400X150 with front/back/left/right walls, floor, and open top
- Target XYZ: [0.675, 0.2, 0.965] m
- Mode: LIFT_ONLY; Yaw = 0; Pitch = 0
- Planning limit: 30 s per stage; maximum 10 attempts
- Full-task trials executed: 20/20
- Dense-valid candidates: 2
- Dense interpolation: revolute <= 0.01 rad, prismatic <= 0.005 m
- Collision detector/distance source: MoveIt FCL PlanningScene (`distanceRobot`, `distanceSelf`)
- q_open = 0.044 m is a simulation planning aperture, not a hardware-validated fully-open value.

- Fixed grasp: `KINEMATIC_REFERENCE_GRASP_50MM`, height=0.050 m, q_contact=0.0327612 m
- Qualification: KINEMATIC_PATH_PLANNING_GRASP_ONLY; NOT_FORCE_CLOSURE_VALIDATED; NOT_HARDWARE_GRASP_STABILITY_VALIDATED
- Task stages: TOP_APPROACH, VERTICAL_DESCENT, GRASP, LIFT_CLEAR, TRANSFER_OUTSIDE
- Minimum object-over-rim vertical clearance: 0.02 m

All-variable minimum margin is expected to be zero because q_open is exactly the independent finger upper bound. Selection therefore uses the minimum margin of the actively planned seven left-arm variables, while still requiring exact bounds validity for all 19 independent variables.

| Trial | Lift | Active margin | Environment clearance | Self clearance | Path length | Planning ms |
|---:|---:|---:|---:|---:|---:|---:|
|4|0.35|0|0.000109414|0.00065926|16.617|3637.91|
|9|0.25|0|0.000581084|0.00065926|16.2958|3278.47|

Result: FAILED_TO_OBTAIN_FIVE_DENSE_VALID_CANDIDATES. No reference was selected.

## Kinematic grasp approval retained

- Name: `KINEMATIC_REFERENCE_GRASP_50MM`
- Status: `SELECTED_FOR_KINEMATIC_PATH_PLANNING`
- Grasp height: 0.050 m above the object bottom
- q_open: 0.044 m
- q_contact_50mm: 0.0327611885070801 m
- Finger-floor clearance: 0.002539612 m
- Actual inner-face overlap: 0.0124168090820314 m
- Symmetric object ±Y contact and measured penetration: 0 m
- The prior 15 mm development criterion remains preserved in its original audit and was not rewritten.
- `KINEMATIC_PATH_PLANNING_GRASP_ONLY`
- `NOT_FORCE_CLOSURE_VALIDATED`
- `NOT_HARDWARE_GRASP_STABILITY_VALIDATED`

## Trial outcome classification

- TOP_APPROACH planning failure: trials 1, 8, 15 (3 trials)
- VERTICAL_DESCENT planning failure: trials 3, 5, 6, 7, 11, 13, 14, 18, 19, 20 (10 trials)
- LIFT_CLEAR planning failure: trials 2, 10, 16, 17 (4 trials)
- Full-task plan rejected by dense validation: trial 12 (1 trial)
- Full-task and dense validation passed: trials 4 and 9 (2 trials)
- Observed exact FCL contact pairs in rejected planning/post-validation output:
  - `box_front_wall` ↔ `openarm_left_left_finger`
  - `box_left_wall` ↔ `openarm_left_left_finger`
  - `box_left_wall` ↔ `openarm_left_right_finger`
  - `target_object` ↔ `openarm_left_right_finger`
- Trial 12 dense rejection: `FINGER_TARGET_PREMATURE_COLLISION:openarm_left_right_finger|target_object`.
- The two dense-valid candidates had no joint-limit violation, no self collision, no robot-box collision, and no attached-object-box collision. Their endpoint orientation errors were all below 0.00026 rad, hence below the configured 0.03 rad limit.
- Both dense-valid candidates had active-joint margin 0. No final candidate was selected because the required five-candidate comparison set was not obtained and a non-boundary alternative was unavailable.
- Initial Lift candidates covered 0.20–0.50 m. During planning, Lift was an active degree of freedom in `left_arm_with_torso`; Yaw and Pitch were constrained to exactly 0. No single final Lift value was selected.

## Safety and termination

- Trajectory execution was never requested.
- No controller, ros2_control node, hardware interface, Serial/CAN/USB node, or real OpenArm driver was run.
- RViz was not started because no reference trajectory was selected.
- The generator finished all 20 trials and wrote/closed the trial and audit files before returning exit code 2 (`insufficient valid candidates`).
- During subsequent SIGINT cleanup, `move_group` emitted the known class-loader unload warning and exited with SIGSEGV (-11). This occurred after output persistence and does not change any planning result.

# Stage-constrained top-open reference audit

- Preserved random-reference classification: `FEASIBLE_BUT_NOT_ROBUST_RANDOM_REFERENCE`
- Methods: OMPL TOP_APPROACH; Cartesian VERTICAL_DESCENT; 10-step finger-only GRASP; Cartesian LIFT_CLEAR; OMPL TRANSFER_OUTSIDE.
- Cartesian TCP spacing: <= 0.005 m; dense validation: revolute <= 0.01 rad, prismatic <= 0.005 m.
- Fixed grasp: `KINEMATIC_REFERENCE_GRASP_50MM`; force closure and hardware stability not validated.
- Global SRDF ACM was not changed. Only finger-target contact was task-scoped during GRASP.
- Trials executed: 10/10

|trial|lift|branch|IK seeds|CF branches|result|failure stage|waypoint|fraction|joint margin|self clearance|environment clearance|pairs|
|---:|---:|---:|---:|---:|---|---|---:|---:|---:|---:|---:|---|
|1|0.25|0|100|40|CONTINUOUS_IK_FAILURE|VERTICAL_DESCENT|20|0.558824|0|0.00065926|0.028901||
|2|0.35|0|100|0|INSUFFICIENT_COLLISION_FREE_IK_BRANCHES|TOP_APPROACH|-1|0|inf|inf|inf||
|3|0.3|0|100|77|CONTINUOUS_IK_FAILURE|VERTICAL_DESCENT|30|0.852941|0|0.00065926|0.0112371||
|4|0.2|0|100|20|CONTINUOUS_IK_FAILURE|VERTICAL_DESCENT|10|0.264706|0|0.00065926|0.00129536||
|5|0.4|0|100|0|INSUFFICIENT_COLLISION_FREE_IK_BRANCHES|TOP_APPROACH|-1|0|inf|inf|inf||
|6|0.25|1|100|48|CONTINUOUS_IK_FAILURE|VERTICAL_DESCENT|20|0.558824|0|0.00065926|0.028901||
|7|0.35|1|100|0|INSUFFICIENT_COLLISION_FREE_IK_BRANCHES|TOP_APPROACH|-1|0|inf|inf|inf||
|8|0.3|1|100|64|CONTINUOUS_IK_FAILURE|VERTICAL_DESCENT|30|0.852941|0|0.00065926|0.011237||
|9|0.2|1|100|28|CONTINUOUS_IK_FAILURE|VERTICAL_DESCENT|10|0.264706|0|0.00065926|0.00385471||
|10|0.4|1|100|0|INSUFFICIENT_COLLISION_FREE_IK_BRANCHES|TOP_APPROACH|-1|0|inf|inf|inf||

## Result

`NO_APPROVABLE_STAGE_CONSTRAINED_REFERENCE`

No trajectory execution, controller, ros2_control, or hardware node was used.

## Failure interpretation

- Lift 0.25 m: 100 seeds per trial produced 40 and 48 collision-free TOP_APPROACH branches. The two ranked branches reached only Cartesian fractions 0.558824 and 0.558824; continuous IK first failed at waypoint 20.
- Lift 0.30 m: 77 and 64 collision-free branches. The two ranked branches reached Cartesian fraction 0.852941; continuous IK first failed at waypoint 30.
- Lift 0.20 m: 20 and 28 collision-free branches. Cartesian fraction 0.264706; continuous IK first failed at waypoint 10.
- Lift 0.35 m and 0.40 m: no TOP_APPROACH branch among the 100 tested seeds simultaneously met joint bounds, collision freedom, and the non-boundary active-revolute criterion.
- No stage-constrained trial reached GRASP, LIFT_CLEAR, or TRANSFER_OUTSIDE. Consequently no task-scoped finger contact or target attachment occurred in these ten trials.
- The failure pairs column is empty because the failures were `CONTINUOUS_IK_FAILURE` or insufficient qualifying IK branches, not FCL collision events.
- The minimum joint-margin value of zero in partial trials includes the initial/default planning state; it is not an approved-reference margin. No final reference margin was produced.

## Historical random-reference collision-stage separation

The preserved random-reference terminal and dense-validation logs classified the observed pairs as follows:

- `box_front_wall` ↔ `openarm_left_left_finger`: OMPL TOP_APPROACH/transition path collision.
- `box_left_wall` ↔ `openarm_left_left_finger`: OMPL approach/descent transition collision.
- `box_left_wall` ↔ `openarm_left_right_finger`: OMPL descent/lift transition collision.
- `target_object` ↔ `openarm_left_right_finger`: premature contact during VERTICAL_DESCENT or another non-GRASP state; trial 12 was explicitly rejected by dense validation as `FINGER_TARGET_PREMATURE_COLLISION`.
- Finger-target contact would be intentional only during the ten-step GRASP closure under the task-scoped ACM. No such intentional contact was treated as an environment failure in this run.

## Comparison with preserved random trials 4 and 9

- Random trial 4 used initial Lift 0.35 m and passed because OMPL was allowed to connect the descent and lift goals in joint space; its active-joint margin was zero and minimum environment clearance was 0.109414 mm.
- Random trial 9 used initial Lift 0.25 m and also passed joint-space OMPL descent/lift; its active-joint margin was zero and minimum environment clearance was 0.581084 mm.
- In the stage-constrained run, Lift 0.35 m had no qualifying non-boundary TOP_APPROACH IK branch, while Lift 0.25 m lost continuous seeded IK at descent waypoint 20 (fraction 0.558824).
- Therefore the earlier feasible paths were not reproduced as continuous vertical Cartesian branches. This run does not claim that the saved random trajectories were reconstructed.

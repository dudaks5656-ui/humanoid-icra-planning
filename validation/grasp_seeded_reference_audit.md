# Grasp-seeded reverse-continuation reference audit

- Previous stage-constrained and random-reference artifacts were preserved.
- Discovery direction: GRASP to TOP_APPROACH; accepted states are reversed for VERTICAL_DESCENT.
- Adaptive TCP intervals tested: 0.005, 0.0025, 0.001 m.
- Each Cartesian waypoint uses the previous RobotState as the IK seed.
- Yaw and Pitch remain zero, including OMPL path constraints.
- No Xacro, SRDF/ACM, joint limit, collision mesh, kinematics, or OMPL configuration was changed.

|trial|Lift|branch|seed|GRASP branches|result|failure stage|waypoint|reverse fraction|spacing|joint margin|self clearance|environment clearance|pairs|
|---:|---:|---:|---:|---:|---|---|---:|---:|---:|---:|---:|---:|---|
|1|0.25|0|-1|0|INSUFFICIENT_COLLISION_FREE_GRASP_IK_BRANCHES|GRASP_BRANCH_DISCOVERY|-1|0|0|inf|inf|inf||
|2|0.35|0|82|24|CONTINUOUS_IK_FAILURE|REVERSE_GRASP_TO_TOP_IK|155|0.916667|0.001|inf|inf|inf||
|3|0.3|0|-1|0|INSUFFICIENT_COLLISION_FREE_GRASP_IK_BRANCHES|GRASP_BRANCH_DISCOVERY|-1|0|0|inf|inf|inf||
|4|0.2|0|-1|0|INSUFFICIENT_COLLISION_FREE_GRASP_IK_BRANCHES|GRASP_BRANCH_DISCOVERY|-1|0|0|inf|inf|inf||
|5|0.4|0|19|18|CONTINUOUS_IK_FAILURE|REVERSE_GRASP_TO_TOP_IK|105|0.619048|0.001|inf|inf|inf||
|6|0.25|1|-1|0|INSUFFICIENT_COLLISION_FREE_GRASP_IK_BRANCHES|GRASP_BRANCH_DISCOVERY|-1|0|0|inf|inf|inf||
|7|0.35|1|22|24|CONTINUOUS_IK_FAILURE|REVERSE_GRASP_TO_TOP_IK|155|0.916667|0.001|inf|inf|inf||
|8|0.3|1|-1|0|INSUFFICIENT_COLLISION_FREE_GRASP_IK_BRANCHES|GRASP_BRANCH_DISCOVERY|-1|0|0|inf|inf|inf||
|9|0.2|1|-1|0|INSUFFICIENT_COLLISION_FREE_GRASP_IK_BRANCHES|GRASP_BRANCH_DISCOVERY|-1|0|0|inf|inf|inf||
|10|0.4|1|85|21|CONTINUOUS_IK_FAILURE|REVERSE_GRASP_TO_TOP_IK|22|0.617647|0.005|inf|inf|inf||

## Result

`NO_APPROVABLE_GRASP_SEEDED_REFERENCE`

No synthetic or partially completed trajectory was promoted as a reference.

## Interpretation

- The strongest result was Lift 0.35 m: the 1 mm continuation reached 91.6667% of the GRASP-to-TOP segment and failed at waypoint 155, approximately 14 mm before the top endpoint.
- Lift 0.40 m reached approximately 61.9% before the same continuous-IK failure class.
- Lift 0.25, 0.30, and 0.20 m produced no GRASP branch satisfying collision freedom and the non-boundary active-joint condition among 100 seeds.
- Empty collision-pair fields mean the recorded stop was an IK failure, not an FCL collision rejection.
- `inf` in rejected discovery rows means `NOT_AVAILABLE`: no complete Cartesian segment was accepted from which a reference-path minimum margin could be reported. It must not be interpreted as infinite physical clearance.
- The previous random trials 4 and 9 demonstrated joint-space feasibility, but their saved results were not reconstructed or promoted here.

Trajectory execution, controllers, ros2_control, and hardware nodes were not used.

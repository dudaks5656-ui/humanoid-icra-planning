# Top-open center reachability — corrected grasp geometry

- Scope: explicit-seed IK and static RobotState/FCL checks; no OMPL or trajectory
- q_open: 0.044 m
- q_contact_50mm: 0.0327611885070801 m (`KINEMATIC_CONTACT_CONFIGURATION_FOR_50MM_OBJECT`)
- Selected grasp-center height above object bottom: 0.0474603881835938 m
- Finger-floor clearance: 1.11022302462516e-16 m
- Vertical grasp-face overlap: 0.0499999999999999 m
- GRASP task-local ACM: only both left finger links vs `target_object`; global SRDF unchanged
- Requested Pitch endpoint: 0.785398163397448 rad (45 deg)
- Used Pitch endpoint: 0.785398 rad (URDF upper bound substitution within 1e-6 rad)

|Stage|LIFT_ONLY|LIFT_YAW_PITCH|Proposed only|
|---|---|---|---|
|APPROACH|possible|possible|no|
|PRE_GRASP|possible|possible|no|
|GRASP|possible|possible|no|

## Complete three-stage candidates

- Best LIFT_ONLY: Lift=0.35 m; minimum clearance=1.12331105017738e-06 m
- Best LIFT_YAW_PITCH: Lift=0.2 m, Yaw=-0.0872664625997165 rad, Pitch=0.349065850398866 rad; minimum clearance=7.59580926645853e-06 m

Observed rejected-state pairs: `box_floor<->openarm_left_left_finger;box_floor<->openarm_left_link0;box_floor<->openarm_left_link1;box_floor<->openarm_left_right_finger;box_floor<->openarm_right_link1;box_floor<->waist_pitch_link;box_front_wall<->openarm_left_link0;box_front_wall<->openarm_left_link1;box_front_wall<->openarm_left_link2;box_front_wall<->openarm_left_link3;box_front_wall<->openarm_left_link4;box_front_wall<->openarm_right_link0;box_front_wall<->openarm_right_link1;box_front_wall<->waist_pitch_link;lift_fixed_link<->openarm_right_left_finger;lift_fixed_link<->openarm_right_link5;lift_fixed_link<->openarm_right_link6;lift_fixed_link<->openarm_right_link7;lift_fixed_link<->openarm_right_right_finger;lift_moving_link<->openarm_right_link6`

## Reference-generation gate

LIFT_ONLY has collision-free IK for all three stages. The static IK gate is satisfied, but OMPL was not run in this task.

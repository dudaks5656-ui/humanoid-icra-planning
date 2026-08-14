# Offline reference path audit

- Scene: provisional method-development confined scene
- Target XYZ: [0.50, 0.30, 1.25] m
- Mode: LIFT_ONLY; Yaw = 0; Pitch = 0
- Planning limit: 30 s per stage; maximum 10 attempts
- Full-task trials executed: 10/20
- Dense-valid candidates: 5
- Dense interpolation: revolute <= 0.01 rad, prismatic <= 0.005 m
- Collision detector/distance source: MoveIt FCL PlanningScene (`distanceRobot`, `distanceSelf`)
- q_open = 0.044 m is a simulation planning aperture, not a hardware-validated fully-open value.

All-variable minimum margin is expected to be zero because q_open is exactly the independent finger upper bound. Selection therefore uses the minimum margin of the actively planned seven left-arm variables, while still requiring exact bounds validity for all 19 independent variables.

| Trial | Lift | Active margin | Environment clearance | Self clearance | Path length | Planning ms |
|---:|---:|---:|---:|---:|---:|---:|
|7|0.1|0|0.00767703|0.00065926|5.3793|947.782|
|8|0.15|0|0.00102884|0.00065926|8.25881|518.945|
|5|0|0|0.000677464|0.00065926|13.3846|951.597|
|4|0.15|0|9.69546e-05|0.00065926|6.67032|511.014|
|10|0.05|0|2.54774e-05|0.00065926|12.5827|894.365|

Selected trial: 7
Selected Lift/Yaw/Pitch: 0.1 / 0 / 0
Selected minimum active-joint margin: 0
Selected minimum environment clearance: 0.00767703
Selected minimum self clearance: 0.00065926
Selected joint-space path length: 5.3793
Selected dense states checked: 395
Attached-object/box collision: NOT_DETECTED

All five candidates had the same minimum active-joint margin of 0 because `openarm_left_joint4` and `openarm_left_joint5` reached exact bounds at recorded points. Trial 7 was therefore selected by the next criterion: it had the largest minimum FCL collision clearance among the tied candidates. No value was clamped.

Endpoint accuracy warning: the selected path's maximum TCP position error is 0.0049002835 m, within the configured 0.005 m tolerance. Its maximum recorded orientation error is 0.0377795721 rad at APPROACH, above the configured 0.03 rad value, although MoveIt returned the plan as successful. EXTRACTION is also slightly above at 0.0302320318 rad. This does not invalidate the bounds/collision audit, but the path is retained as a method-development reference and the endpoint orientation must be checked visually in RViz before later experiments.

All five stages are connected by the previous segment's final RobotState. The target remains a world object through GRASP, finger-target contact is allowed only at GRASP, and the target is attached for LIFT and EXTRACTION. No trajectory was executed.

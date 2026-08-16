# Lift-actuated extraction baseline v1 audit

Generated: 2026-08-15T16:56:09+0900

## Existing implementation audit

- v1 `cartesian()` held the trial Lift value and called `setFromIK(left_arm_group_)` for every VERTICAL_DESCENT and LIFT_CLEAR Cartesian Z waypoint.
- v2 `followCartesian()` / `adaptiveContinuation()` likewise kept Lift at the grasp candidate and used sequential Arm IK for reverse descent discovery and attached LIFT_CLEAR.
- v3 set Lift to 0.35 or 0.40 at every layer and generated each Cartesian Z layer with Arm IK.
- torso recovery kept Lift fixed and added Yaw/Pitch while Arm IK still generated Cartesian Z layers.
- The focused boundary audit identified openarm_left_joint3 and openarm_left_joint5 upper-limit termination during that Arm-actuated vertical motion.

## Corrected baseline

Only grasp configuration uses Arm IK. STRICT_ARM_LOCKED then changes only lift_joint at <=1 mm spacing for descent and attached ascent. No OMPL planning request, controller, ros2_control, RViz, hardware, or trajectory execution was used. Force closure is not claimed.

Lift direction audit for +0.001 m: TCP delta xyz = 0 0 -0.001 m.

|Grasp Lift|STRICT result|Descent / ascent (m)|Max Arm delta|j3 / j5 minimum margin|Environment / self minimum clearance|Object-bottom clearance|First failure|Pairs|
|---:|---|---:|---:|---:|---:|---:|---|---|
|0.35|LIFT_ACTUATED_EXTRACTION_SUCCESS|0.17 / 0.17|0|1.20035e-06 / 1.80167e-06|0.00253968 / 0.00065926|0.02|:-1||
|0.4|LIFT_ACTUATED_EXTRACTION_SUCCESS|0.17 / 0.17|0|1.30623e-06 / 1.62137e-06|0.0025398 / 0.00065926|0.02|:-1||

Validation stops when the attached object clears the box top; TRANSFER_OUTSIDE and the complete five-stage experiment are intentionally excluded.

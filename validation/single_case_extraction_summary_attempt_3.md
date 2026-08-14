# Single-case extraction summary

Generated: 2026-08-12T06:28:45+0900

All scene, object, pose, and task-distance values are **PROVISIONAL_DEVELOPMENT_VALUE** and are not final paper data.

## Scene

- Frame: `world`
- Box center [m]: `0.7 0.2 1.12`
- Interior width/depth/height [m]: `0.4 0.45 0.36`
- Wall/floor thickness [m]: `0.025 0.025`
- Target position [m]: `0.5 0.3 1.25`
- Target size [m]: `0.05 0.05 0.08`
- TCP to grasp center [m]: `0.13546`
- Selected pre-grasp clearance [m]: `0.06`
- Insertion/lift [m]: `0.02 0.08`
- Selected extraction clearance [m]: `0.005`
- Box front plane X [m]: `0.475`
- Final extracted object center [m]: `0.445 0.3 1.33`
- Legacy fixed extraction_distance [m]: `0.24` (**DEPRECATED_NOT_USED**)
- Planning attempt ID/time limit/number of attempts: `attempt_3 / 2 s / 2`
- Simulation planning aperture q [m]: `0.044`

## Result

- TORSO_LOCKED: **FAILURE**
- TORSO_LOCKED first failure stage: `GRASP_POSE`
- TORSO_CANDIDATE_SEARCH: **SUCCESS**
- Successful candidate: `yaw_-5_pitch_+0`
- Successful Yaw/Pitch: `-5 deg / 0 deg`
- Executed candidates: 3
- Excluded out-of-limit candidates: 0 (no clamping)
- Desired locked-failure / torso-recovery case: **YES**

## Safety and limitations

No trajectory was executed. No ros2_control, controller, hardware interface, or real robot node was used. The target was attached only as a PlanningScene collision object; object dynamics, physical contact, grasp closure, and real grasp success were not validated. Both gripper independent joints remained at the simulation planning aperture 0.044 m; this is not asserted to be a hardware-validated fully-open position. The target remained a world collision object through GRASP_POSE and was attached atomically for LIFT and EXTRACTION.

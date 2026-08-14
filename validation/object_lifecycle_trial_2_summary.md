# Single-case extraction summary

Generated: 2026-08-12T05:49:01+0900

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
- Insertion/lift/extraction [m]: `0.02 0.08 0.24`
- Simulation planning aperture q [m]: `0.044`

## Result

- TORSO_LOCKED: **FAILURE**
- TORSO_LOCKED first failure stage: `GRASP_POSE`
- TORSO_CANDIDATE_SEARCH: **FAILURE**
- Successful candidate: `none`
- Executed candidates: 25
- Excluded out-of-limit candidates: 5 (no clamping)
- Desired locked-failure / torso-recovery case: **NO**

## Safety and limitations

No trajectory was executed. No ros2_control, controller, hardware interface, or real robot node was used. The target was not attached, and object dynamics, contact, grasp closure, and real grasp success were not validated. Both gripper independent joints remained at the simulation planning aperture 0.044 m; this is not asserted to be a hardware-validated fully-open position. The target remained a world collision object through GRASP_POSE and was attached atomically for LIFT and EXTRACTION.

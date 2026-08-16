# Paper result RViz replay v1 — validation audit

Generated: 2026-08-16 KST

## Scope

- Source dataset: `/home/openarm/humanoid_sim_ws/validation/paper_main_simulation_dataset_v1/run_20260815_223216`
- Replay case: `LOCKED_COMMON_SUCCESS`
- Unique key: `PHASE1|R8_0.020|R8|0.020|0.400|LOCKED|0`
- Planning cases re-executed: **false**
- IK, OMPL, MoveGroup planning, controller, ros2_control, hardware, and trajectory execution used: **false**

## Stored-data and RobotModel checks

- `all_case_results.csv` remained at SHA-256 `f95048fe3e48ebffd3d1f255242a22ea03f8505dd59aa19ae3658aef8b696cf8` and mtime `2026-08-16T10:21:14.220782447+09:00`.
- All six automatically selected keys matched `representative_case_audit.yaml`.
- CSV Lift, waist Yaw/Pitch, Arm 1–7, and finger joint names exist in the expanded current URDF.
- TCP link `openarm_left_hand_tcp` exists.
- Lift axis is world `(0, 0, -1)`: increasing q moves downward and decreasing q moves upward.
- Successful rows were checked against the current URDF limits; the physical-envelope failure intentionally has no stored robot posture.

## LOCKED replay checks

- Headless publisher replay reached `HOLDING_FINAL_CLEARANCE` using the same messages as the RViz launch.
- Stored/displayed joint maximum error: `0.0 rad`.
- Arm maximum change during Lift motion: `0.0 rad`.
- Lift q: `0.23 → 0.40 → 0.23 m`.
- Dense replay step: `1 mm`.
- TCP maximum XY error: `0.0 m`.
- TCP maximum orientation error: `0.0 rad`.
- TCP Z/lift consistency error after timestamp matching: `2.220446049250313e-16 m`.
- Expected attached-object bottom clearance above the box top at the final state: `0.020 m`.
- The target is a world collision object before grasp and an `AttachedCollisionObject` on `openarm_left_hand_tcp` after grasp.
- Each transition has a 2 s hold; the final clearance state is held indefinitely during the interactive launch.

## Visual status

- Two requests to start the actual RViz GUI were not executed because the managed GUI permission review timed out; no RViz process was created.
- Consequently, direct on-screen confirmation of mesh penetration and camera appearance remains pending. No visual success was claimed.
- The launch, RViz configuration, MarkerArray, PlanningScene, JointState, attachment, text overlay, and six-case launch argument paths are implemented and statically validated.
- The gripper-envelope failure replay uses the stored swept-envelope AABB shifted to the selected target and highlights `box_front_wall` in red; it does not invent a robot posture for the NaN failure row.

## Shutdown

- Both headless verification runs were limited to `LOCKED_COMMON_SUCCESS` and were stopped with SIGINT.
- After the replay SIGINT handling fix, robot_state_publisher and replay node exited cleanly.

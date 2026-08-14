# Lift-only versus Lift-Yaw-Pitch boundary search

Generated: 2026-08-12T06:56:31+0900

- Previous `TORSO_LOCKED` results are preserved as **LEGACY_FIXED_LIFT** because Lift was fixed at q=0.
- Runtime URDF Lift limit: `0 .. 0.7 m`; q=0 is top and positive motion is downward.
- Local Lift search candidates: 0 0.05 0.1 0.15 m.
- IK multistart: `30` explicit seeds per stage; RNG base seed `20260812`.
- Planning time/attempts are identical in both modes: `2 s / 2`.
- Search phase: `SEARCH_EXHAUSTED`.
- Recovery found: **NO**.

## Search result

- Evaluated targets: `35` (`11` Y-boundary targets plus `24` X-Y targets using the six feasible Y values).
- Geometrically feasible: `30`; geometrically infeasible: `5`.
- The infeasible boundary is `Y >= 0.330 m` for the tested X=0.50 slice. Every available IK solution at the affected goal pose contained `box_left_wall | openarm_left_left_finger`; OMPL was not called for these targets.
- LIFT_ONLY completed all five stages at `27/30` feasible targets.
- LIFT_YAW_PITCH completed all five stages at `30/30` feasible targets.
- The remaining three LIFT_ONLY failures occurred at `[0.50,0.300,1.25]`, `[0.525,0.305,1.25]`, and `[0.575,0.315,1.25]`, but every stage had collision-free IK among the 30 seeds. They were OMPL failures, not `STRUCTURAL_STAGE_FAILURE`, and therefore were not accepted as torso recovery cases.
- No target met `LIFT_ONLY structural failure` plus `LIFT_YAW_PITCH five-stage success`; orientation and robot configuration were not changed.

## Output integrity

- `target_boundary_search.csv`: 36 lines, 17 fields per row, no malformed rows.
- `lift_only_vs_lift_yaw_pitch.csv`: 160 lines, 16 fields per row, no malformed rows.
- `ik_multistart_audit.csv`: 12,601 lines (12,600 seed records), 17 fields per row, no malformed rows.
- `geometric_infeasibility_map.csv`: 36 lines, 8 fields per row, no malformed rows.
- Every CSV ended with newline byte `0x0A`; all streams were flushed and closed before process shutdown.

No trajectory was executed. No controller, ros2_control, hardware interface, serial, CAN, USB, or real robot node was used. Geometry and experiment values remain provisional.

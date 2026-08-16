# Box target Yaw/Pitch feasibility pilot v1

Generated: 2026-08-15T19:41:23+0900

- Scope: 9 target positions x 2 Lift values x 2 modes = 36 results.
- LOCKED success: 4/18.
- Yaw/Pitch posture-selection success: 4/18.
- Qualified YAW_PITCH_FEASIBILITY_RECOVERY cases: 0.
- Target positions were calculated from the 0.400 x 0.600 m inner bounds, the 0.025 m target half-size, and a 0.025 m target-to-wall clearance.
- Arm and selected Yaw/Pitch were locked after grasp selection; only lift_joint generated vertical descent/ascent.
- No move_group, OMPL, controller, ros2_control, hardware, trajectory execution, or RViz was started. Force closure is not claimed.
- The run stops after the attached object reaches 20 mm clearance above the box top.
- URDF Yaw bounds [rad]: [-0.174533, 0.174533]; Pitch bounds [rad]: [-0.174533, 0.785398]. Requested endpoints are clamped to these exact URDF bounds.

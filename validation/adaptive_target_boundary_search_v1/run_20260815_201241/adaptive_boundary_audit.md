# Adaptive target boundary search v1

Generated: 2026-08-15T20:51:25+0900

- Scope: 8 center rays x 2 fixed Lift values; adaptive 20/5/1 mm LOCKED boundary pilot.
- Yaw/Pitch was evaluated only at the last LOCKED success, first failure, up to two post-failure points, and a sharp-margin point.
- Qualified YAW_PITCH_FEASIBILITY_RECOVERY boundaries: 10.
- Rays containing a GRIPPER_ENVELOPE_INFEASIBLE sample: 6.
- The swept envelope uses actual collision shapes and collision-origin transforms of openarm_left_link7 and both finger links across open-to-q_contact motion.
- Arm and selected torso posture remained fixed during <=1 mm Lift-only descent/ascent; attached-object clearance target was 20 mm.
- No move_group, OMPL, controller, ros2_control, hardware, trajectory execution, or RViz was started.

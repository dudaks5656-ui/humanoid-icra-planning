# Differential planning-budget analysis

Generated: 2026-08-12T07:29:15+0900

Targets were extracted automatically from `/home/openarm/humanoid_sim_ws/validation/lift_only_vs_lift_yaw_pitch.csv`: LIFT_ONLY had no full five-stage success, collision-free IK existed, and LIFT_YAW_PITCH succeeded.

Each target used both modes, budgets 2/5/10 s, 20 repeats, and 2 planning attempts. No fixed OMPL seed is claimed. The explicit seed applies only to the deterministic 30-start IK audit.

| Target | XYZ (m) | Proposed Lift/Yaw/Pitch | Classification | LIFT_ONLY rates 2/5/10 s | Proposed rates 2/5/10 s |
|---|---|---|---|---|---|
|target_1|0.5 0.3 1.25|0 / 0 / 0|NO_MEANINGFUL_TORSO_ADVANTAGE|0.55/0.3/0.6|0.4/0.4/0.45|
|target_2|0.525 0.305 1.25|0 / 0 / 0|NO_MEANINGFUL_TORSO_ADVANTAGE|0.55/0.5/0.45|0.45/0.25/0.5|
|target_3|0.575 0.315 1.25|0 / -0.0872665 / 0|NO_MEANINGFUL_TORSO_ADVANTAGE|0.6/0.6/0.6|0.35/0.3/0.55|

Classification thresholds used for this focused diagnostic: short-budget convergence requires >=0.8 at 5 or 10 s; persistent advantage requires baseline 10 s <0.5 and Proposed advantage >=0.25. Other outcomes are NO_MEANINGFUL_TORSO_ADVANTAGE. These are time-budgeted planning outcomes, not structural impossibility.

Interpretation notes:

- `target_1` and `target_2` were selected by the required historical CSV filter, but their previously successful Proposed candidates have Lift/Yaw/Pitch = `0/0/0`. Their repeated LIFT_ONLY and LIFT_YAW_PITCH tests therefore have no functional torso-pose difference.
- `target_3` is the only extracted target with a non-zero Proposed torso pose (Yaw = -5 degrees). At 10 s its LIFT_ONLY success rate was 0.60 and its LIFT_YAW_PITCH success rate was 0.55, so the repeated trial does not support a meaningful torso advantage for this target.
- Across all stage rows, 170 failures were MoveIt planning failures and 22 returned trajectories were rejected by the independent full-trajectory check for a joint-limit violation. Those rejected trajectories were not counted as successes.
- No recorded stage reported an attached-target/box collision. Successful trials therefore did not pass through a detected attached-object/box collision.

Output integrity: 360 unique trials were saved; every one of the 18 target/mode/budget conditions contains exactly 20 repeats. The trial CSV has 23 fields on every row and the aggregate CSV has 18 fields on every row.

No trajectory was executed. No controller, ros2_control, hardware interface, serial, CAN, USB, or real robot node was used.

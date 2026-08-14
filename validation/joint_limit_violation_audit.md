# Joint-limit violation audit

## Result

The 360-repeat result contains exactly 22 stage summaries marked `TRAJECTORY_JOINT_LIMIT`. The original `moveit_msgs/RobotTrajectory` point arrays for those rejected stages were not written to disk. The saved rows contain the target, mode, budget, repeat, stage and failure label, but contain no joint names, joint values, point indices or bound excess.

Consequently, the original 22 cases cannot be assigned honestly to `NUMERICAL_TOLERANCE_ONLY`, `SMALL_BUT_REAL_VIOLATION` or `INVALID_TRAJECTORY`. Re-running OMPL would produce new stochastic trajectories, not reconstruct the original 22. The audit CSV therefore records `NOT_DETERMINABLE_RAW_TRAJECTORY_NOT_PRESERVED` and leaves unavailable numeric fields explicit rather than fabricating or clamping values.

## What can be established from the implementation

- The planning group was `left_arm`; its trajectory contains the seven independent revolute joints `openarm_left_joint1` through `openarm_left_joint7`.
- Lift, Yaw, Pitch and both independent finger joints were fixed in these plans and were valid at the stage start.
- `openarm_left_finger_joint2` and `openarm_right_finger_joint2` are mimic joints and were not independent trajectory variables.
- The audited model contains no continuous joints.
- The rejection occurred when `RobotState::satisfiesBounds(whole_body)` returned false after applying a trajectory point. This narrows the likely source to an independent bounded trajectory variable, but does not identify which left-arm joint or its excess without the discarded point array.

## Limits used by the left-arm planning variables

| Joint | Kind | Lower | Upper |
|---|---|---:|---:|
| openarm_left_joint1 | independent revolute | -3.490659 | 1.396263 |
| openarm_left_joint2 | independent revolute | -3.3161253267948965 | 0.17453267320510335 |
| openarm_left_joint3 | independent revolute | -1.570796 | 1.570796 |
| openarm_left_joint4 | independent revolute | 0.0 | 2.443461 |
| openarm_left_joint5 | independent revolute | -1.570796 | 1.570796 |
| openarm_left_joint6 | independent revolute | -0.785398 | 0.785398 |
| openarm_left_joint7 | independent revolute | -1.570796 | 1.570796 |

No limit was widened and no Xacro, SRDF, joint-limit, kinematics or OMPL file was modified. The new offline reference generator stores every accepted trajectory point and rejects any candidate with an exact bounds violation; it never clamps a value into range.

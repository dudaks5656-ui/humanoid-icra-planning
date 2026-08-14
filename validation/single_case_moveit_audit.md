# Single-case extraction: existing MoveIt audit

Date: 2026-08-12 (Asia/Seoul)

## Existing configuration

- Robot Xacro: `/home/openarm/humanoid_sim_ws/src/humanoid_sim_description/urdf/humanoid_sim.urdf.xacro`
- SRDF: `/home/openarm/humanoid_sim_ws/src/humanoid_sim_moveit_config/config/humanoid_sim.srdf`
- Planning-only launch: `/home/openarm/humanoid_sim_ws/src/humanoid_sim_moveit_config/launch/planning_only.launch.py`
- Planning pipeline: `ompl_interface/OMPLPlanner`
- Default planner for every group: `RRTConnectkConfigDefault` (`geometric::RRTConnect`)
- Baseline `longest_valid_segment_fraction`: `0.01`
- Fine comparison profile: `0.0025`
- Kinematics solver: `kdl_kinematics_plugin/KDLKinematicsPlugin`
- KDL plugin library present: `/opt/ros/humble/lib/libmoveit_kdl_kinematics_plugin.so`

Planning groups: `torso`, `left_arm`, `right_arm`, `left_gripper`, `right_gripper`, `left_arm_with_torso`, `right_arm_with_torso`, `dual_arm`, `dual_arm_with_torso`, `whole_body`.

The experiment uses `left_arm`. The matching right-side group is `right_arm`.

## End effectors

- Left SRDF end-effector parent: `openarm_left_link7`; left TCP target link: `openarm_left_hand_tcp`
- Right SRDF end-effector parent: `openarm_right_link7`; right TCP target link: `openarm_right_hand_tcp`

## Torso joints from the expanded URDF

| Joint | Type | Axis | Lower | Upper |
|---|---|---|---:|---:|
| `lift_joint` | prismatic | `0 0 -1` | 0.0 m | 0.7 m |
| `waist_yaw_joint` | revolute | `0 0 1` | -0.174533 rad | 0.174533 rad |
| `waist_pitch_joint` | revolute | `0 1 0` | -0.174533 rad | 0.785398 rad |

## `whole_body` independent variables (19)

1. `lift_joint`
2. `waist_yaw_joint`
3. `waist_pitch_joint`
4. `openarm_left_joint1`
5. `openarm_left_joint2`
6. `openarm_left_joint3`
7. `openarm_left_joint4`
8. `openarm_left_joint5`
9. `openarm_left_joint6`
10. `openarm_left_joint7`
11. `openarm_right_joint1`
12. `openarm_right_joint2`
13. `openarm_right_joint3`
14. `openarm_right_joint4`
15. `openarm_right_joint5`
16. `openarm_right_joint6`
17. `openarm_right_joint7`
18. `openarm_left_finger_joint1`
19. `openarm_right_finger_joint1`

The two `finger_joint2` joints are mimic/passive joints and are not independent variables.

## Safety scope

The existing launch has `allow_trajectory_execution=false`, controller management disabled, and `MoveGroupExecuteTrajectoryAction` disabled. This experiment preserves those settings and calls planning only. It does not modify the Xacro, SRDF, joint limits, validated mount configuration, or ACM.

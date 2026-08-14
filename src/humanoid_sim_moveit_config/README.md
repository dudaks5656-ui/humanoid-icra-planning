# Humanoid simulation MoveIt configuration

This package is planning-only. It does not start `ros2_control`, a controller
spawner, a hardware interface, or any OpenArm/AMR driver. The unchanged robot
description is loaded from:

`/home/openarm/humanoid_sim_ws/src/humanoid_sim_description/urdf/humanoid_sim.urdf.xacro`

The URDF root remains `world`; the existing fixed `world_to_base_joint` is not
part of any planning group. There is no SRDF virtual joint.

## Planning-only launch

For the normal interactive RViz session, use the workspace helper. It sources
ROS 2 and the workspace, fixes the isolated simulation DDS settings, and starts
the complete STL/collision/bounds/MoveIt stack:

```bash
cd /home/openarm/humanoid_sim_ws
./run_humanoid_moveit.sh
```

Use `./run_humanoid_moveit.sh fine` only when the finer provisional OMPL
collision-sampling profile is intentionally required.

The equivalent manual command is:

```bash
source /opt/ros/humble/setup.bash
source /home/openarm/humanoid_sim_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
ros2 launch humanoid_sim_moveit_config planning_only.launch.py ompl_profile:=baseline
```

The comparison profile is selected with `ompl_profile:=fine`. Baseline uses
`longest_valid_segment_fraction=0.01`; fine uses the provisional evaluation
value `0.0025`. Neither value is a final safety certification. Trajectory
execution is false and the ExecuteTrajectory capability is disabled.

## Full RobotState collision checker

```bash
ros2 launch humanoid_sim_moveit_config collision_state_checker.launch.py
```

Publish a complete `sensor_msgs/msg/JointState` to
`/collision_state_input`. All 19 independent variables are required:

1. `lift_joint`
2. `waist_yaw_joint`
3. `waist_pitch_joint`
4. `openarm_left_joint1` ... `openarm_left_joint7`
5. `openarm_left_finger_joint1`
6. `openarm_right_joint1` ... `openarm_right_joint7`
7. `openarm_right_finger_joint1`

The two `finger_joint2` variables are mimic joints and must not be supplied as
independent inputs. Set `JointState.header.frame_id` to the sampling method,
for example `unit_test_boundary`, `random`, `quasi_random`, `ik`, or
`ompl_trajectory`. These labels classify samples; they do not define safe
angle tables.

The checker subscribes to `/monitored_planning_scene` for environment objects
and to `/display_planned_path` for planned trajectories. Every source segment
is rechecked at provisional coarse/medium/fine subdivisions of 1/5/20. Results
are written to:

- `/home/openarm/humanoid_sim_ws/validation/collision_results.csv`
- `/home/openarm/humanoid_sim_ws/validation/trajectory_validation.csv`

## Self-collision audit

```bash
ros2 launch humanoid_sim_moveit_config self_collision_matrix_audit.launch.py sample_count:=10000
```

Sampling never disables a pair merely because it was not observed in
collision. Only direct adjacent pairs listed in the SRDF are disabled.

# Humanoid single-case extraction experiment

This package is isolated from the real OpenArm workspace. It performs planning only and never calls trajectory execution.

The scene and target values in `config/confined_scene.yaml` are `PROVISIONAL_DEVELOPMENT_VALUE`. They are not final paper data and do not validate grasp physics or object dynamics.

The experiment compares:

- `TORSO_LOCKED`: Lift fixed, Waist Yaw = 0, Waist Pitch = 0, left-arm planning only.
- `TORSO_CANDIDATE_SEARCH`: Lift fixed; discrete Yaw/Pitch candidates are set as fixed start-state posture while the same left-arm stages are planned.

Both independent gripper variables are set to `0.011 m` in every planning start state. This is only the currently collision-free planning aperture; no physical open/closed meaning is asserted.

No `execute()`, controller, ros2_control node, hardware interface, serial/CAN/USB access, or real-robot launch is used.

## Preserved workspace-boundary visualization

The existing `target_boundary_search.csv` can be displayed without rerunning or
overwriting the experiment:

```bash
cd /home/openarm/humanoid_sim_ws
./run_humanoid_workspace_boundary.sh
```

This is only the sampled XY task-space slice at `z=1.25 m`, not a complete 3D
workspace or safety-certified envelope. Green samples are full-task LIFT_ONLY
successes; cyan samples succeeded in the recorded torso-candidate run but are
not proof of structural recovery; red samples are geometrically infeasible due
to direct collision. The original box and target geometry are shown for context.

The simulation-only robot visuals use a bright aluminum-gray material so the
hardware remains distinguishable from the red infeasible samples. MoveIt still
uses red for a link that is actively colliding; that diagnostic override is
intentionally preserved.

## Build

```bash
bash --noprofile --norc
source /opt/ros/humble/setup.bash
source /home/openarm/humanoid_sim_ws/install/setup.bash
cd /home/openarm/humanoid_sim_ws
colcon build --symlink-install --packages-select humanoid_extraction_experiments
```

## Planning-only run

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
ros2 launch humanoid_extraction_experiments single_case_extraction.launch.py
```

`Plan & Execute` and trajectory execution are disabled. The successful trajectory, if found, is only published for RViz visualization.

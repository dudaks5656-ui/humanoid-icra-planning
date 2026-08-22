# Runtime Grasp TCP Consistency Audit

## Runtime Model Chain

- workspace launch: `src/fixed_base_workspace_analysis/launch/fixed_base_workspace.launch.py` (the fine and DOF-ablation launches use the same description entry point)
- robot_description xacro: `src/humanoid_sim_description/urdf/humanoid_sim.urdf.xacro`
- arms adapter: `src/humanoid_sim_description/urdf/openarm_arms_adapter.xacro`
- ee macro: `src/openarm_description/assets/robot/openarm_v1.0/urdf/ee/ee_dispatcher.xacro` -> `parallel_link_ee.xacro`
- runtime-chain inclusion of the sim arms adapter: PASS
- `humanoid_description/urdf/openarm_v10_arms_adapter.xacro` in runtime chain: NO

## Before

LEFT:

- frame: `openarm_left_hand_tcp`
- parent: `openarm_left_link7`
- xyz: `[0, 0, 0]` m
- rpy: `[0, 0, 0]` rad

RIGHT:

- frame: `openarm_right_hand_tcp`
- parent: `openarm_right_link7`
- xyz: `[0, 0, 0]` m
- rpy: `[0, 0, 0]` rad

## After

LEFT:

- frame: `openarm_left_hand_tcp`
- parent: `openarm_left_link7`
- xyz: `[0, 0, 0.0345]` m
- rpy: `[0, 0, 0]` rad

RIGHT:

- frame: `openarm_right_hand_tcp`
- parent: `openarm_right_link7`
- xyz: `[0, 0, 0.0345]` m
- rpy: `[0, 0, 0]` rad

## FK

- left delta: `0.034500000000 m`
- right delta: `0.034500000000 m`
- left orientation delta: `0.000000000000 rad`
- right orientation delta: `0.000000000000 rad`
- parent-child chains unchanged: PASS

The FK comparison used the same zero/default movable-joint configuration in the pre-change and post-change expanded runtime URDFs. The complete root-to-TCP fixed/movable origin chains were composed; the result is independent of the chosen common movable-joint state because the intentional change is a 34.5 mm TCP-local translation.

## Double Offset Check

- expected total offset: `0.034500 m`
- left actual local offset: `0.034500 m`
- right actual local offset: `0.034500 m`
- status: PASS

The previous `humanoid_description/urdf/openarm_v10_arms_adapter.xacro` correction is not included by `humanoid_sim.urdf.xacro`. The shared EE dispatcher only forwards the adapter argument; it does not add a second offset. Both final runtime TCP joints contain exactly one `0.0345 m` translation.

## Protected Model Check

- xacro parse: PASS
- `check_urdf`: PASS
- runtime `robot_description` expansion: PASS
- generated URDF differs only at the two TCP fixed-joint `xyz` attributes: PASS
- collision geometry unchanged: PASS (25 elements)
- visual geometry unchanged: PASS (25 elements)
- inertial unchanged: PASS (20 elements)
- non-TCP joint origin unchanged: PASS
- joint axis unchanged: PASS (21 elements)
- joint limits unchanged: PASS (21 elements; MoveIt YAML hash unchanged)
- arm mount transforms unchanged: PASS
- SRDF/ACM unchanged: PASS
- kinematics unchanged: PASS
- OMPL unchanged: PASS
- robot model files changed in this correction: only `src/humanoid_sim_description/urdf/openarm_arms_adapter.xacro`
- runtime TCP consistency correction outside the two requested call arguments: `0`

## Final

- Runtime TCP consistency: PASS
- Ready for FK workspace analysis: YES

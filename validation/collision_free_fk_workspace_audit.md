# Collision-Free FK Workspace Audit

## Definition

- Workspace type: collision-free configuration FK workspace
- Source commit: `ca48b5e5f6f9135d89b1700e595a6ff7bc157306`
- Analysis hand: LEFT (same convention as the prior workspace research)
- Base frame: `base_link`; AMR fixed
- TCP: `openarm_left_hand_tcp`, parent `openarm_left_link7`, xyz `0;0;0.0345`, RPY `0;0;0`
- Right TCP preflight: `openarm_right_hand_tcp`, parent `openarm_right_link7`, xyz `0;0;0.0344999999999998`, RPY `0;0;0`
- Environment collision objects: none
- Sampling: deterministic Halton nested state pool; seed `20260819`
- Collision checks: `36000` (hard cap 40000)
- Quantitative occupancy: validated endpoints at 10 mm; sensitivity at 5/10/15 mm
- Convex hull: not used; empty cells and projection holes are preserved

## Nested construction

- C0 = BASE valid states
- C1 = all C0 states + YAW enrichment
- C2 = all C0 states + PITCH enrichment
- C3 = all C0/C1/C2 source states + COMBINED enrichment
- State-level and 5/10/15 mm occupancy inclusion checks: **PASS**
- Differential partition identity and disjointness at every tested resolution: **PASS**

## Results at 10 mm

|Configuration|Nested valid states|Collision rejects (new attempts)|X max [m]|Y range [m]|Z range [m]|Occupied measure [m^3]|Front area [m^2]|Right area [m^2]|
|---|---:|---:|---:|---:|---:|---:|---:|---:|
|LIFT_ONLY|8778|3222|0.712603|-0.120742..0.620870|0.552677..2.000761|0.008635|0.456000|0.508600|
|LIFT_YAW|13896|1882|0.738626|-0.134993..0.620870|0.552677..2.000761|0.013216|0.528300|0.624900|
|LIFT_PITCH|11818|3960|0.774114|-0.120742..0.620870|0.534513..2.000761|0.011361|0.509600|0.589800|
|LIFT_YAW_PITCH|21897|5039|0.822354|-0.135145..0.627570|0.534513..2.000761|0.020155|0.608800|0.750900|


The occupied measure is a sampling- and resolution-dependent FK endpoint occupancy estimate, not a convex-hull volume.

## Convergence

- Milestones: 25%, 50%, 75%, 100%
- Final criterion: extent change <= 0.02 m and occupancy growth <= 0.05
- Judgment: **NOT CONVERGED**
- LIFT_ONLY: final extent change `0.003520 m`, occupancy change `0.328257`
- LIFT_YAW: final extent change `0.001788 m`, occupancy change `0.326641`
- LIFT_PITCH: final extent change `0.004309 m`, occupancy change `0.328306`
- LIFT_YAW_PITCH: final extent change `0.043197 m`, occupancy change `0.323114`


## Representative cross-check

- Points tested: 10 (hard cap 20)
- All selected endpoints retain their exact generating joint state, satisfying bounds and self-collision checks: **PASS**
- The exact source state is a constructive IK-existence witness; no Cartesian grid or broad IK sweep was executed.
- This does not claim a collision-free path from neutral to every state.

## Old versus corrected TCP

- Old results are retained read-only and are not mixed into the new quantitative estimate.
- Old method: directed boundary sweep using the pre-correction TCP.
- New method: nested collision-free joint-state sampling and FK of the corrected grasp TCP.
- A 34.5 mm link-local offset does not imply a constant +34.5 mm base-frame X change because TCP orientation varies.

## Integrity and safety

- `src/humanoid_sim_description/urdf/openarm_arms_adapter.xacro` SHA-256: `5a343f4b8363727cab9fa15f7c1d71011867ab6d7012693911735780eb2baa1a`
- `src/humanoid_description/urdf/openarm_v10_arms_adapter.xacro` SHA-256: `ae6170cebf57dc5cde61d834fc5b07b932a0941b6cf6a4de35c67d1160f061ee`
- `src/openarm_description/assets/robot/openarm_v1.0/urdf/ee/ee_dispatcher.xacro` SHA-256: `0424ccb1962911f5364b30c494972d3a92938f102e4f3c923e920c21c2c374ed`
- `src/humanoid_sim_moveit_config/config/humanoid_sim.srdf` SHA-256: `7892b122fe28e91650f8919a66c256ebc55d3b34472e006701a63cadab5484e8`
- `src/humanoid_sim_moveit_config/config/joint_limits.yaml` SHA-256: `1e851fa791dab1c95eeba4a698e620c0334afd12a0c9964b774f24868d03ccfb`
- `src/humanoid_sim_moveit_config/config/kinematics.yaml` SHA-256: `7a825c85e6d3bfa06b3ff6d195b3aced8c490d6132d8d94e2d0b01f52e73204e`
- `src/humanoid_sim_moveit_config/config/ompl_planning.yaml` SHA-256: `3e47c074ffcb9a72ea62e8821a8923f80c7f346e246993b3131038380aaf1335`

- Every workspace endpoint originates from a state with valid joint limits and no self-collision: **PASS**
- Deterministic sequence contract: fixed Halton index, dimensions, seed, and hard caps recorded in metadata.
- Independent repeat run: **PASS**. The complete state CSV was byte-identical (`d2a118f18ac48cef58c39d6c4a8f415c3f015270050685bd201e18b6525f3463`); metadata excluding wall time was identical.
- IK grid workspace rerun: **NO**
- OMPL/path planning: **NO**
- Trajectory execution: **NO**
- Controller/ros2_control/hardware: **NO**
- AMR motion: **NO**

## Interpretation boundary

This dataset proves that a collision-free robot configuration exists for each recorded endpoint. It does not prove that every endpoint has a collision-free trajectory from a neutral/reference pose. Representative path feasibility remains a separate task-level MoveIt experiment.

## Presentation

- Ten PNG figures were generated and visually inspected: PASS
- Optional MP4: not generated because neither `ffmpeg` nor `ffprobe` is installed in the current environment; no package installation was attempted.

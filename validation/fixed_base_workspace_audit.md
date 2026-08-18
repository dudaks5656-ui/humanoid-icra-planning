# Fixed-base manipulation workspace audit

Generated: 2026-08-18T10:14:00Z

## Scope and model contract

- Experiment: IK-based, self-collision-aware, fixed-base workspace characterization.
- AMR/base fixed: **yes**; no base variables are sampled or modified.
- Robot model frame: `world`
- Base frame and coordinate convention: `base_link`; +X forward, +Y left, +Z up.
- TCP frame: `openarm_left_hand_tcp`
- TCP orientation source: `validation/reference_grasp_50mm.yaml`; quaternion xyzw = [0, 1, 0, 0]
- Orientation tolerance: 0.03 rad.
- IK method: bounded discrete torso candidates plus the existing `left_arm` KDL IK solver.
- LIFT_ONLY fixes waist yaw/pitch to exactly 0 for every seed.
- LIFT_YAW_PITCH samples the same lift-only postures first, then a deterministic bounded torso lattice.
- Environment objects: none. Existing SRDF ACM is used unchanged.
- Controller: none; ros2_control: none; trajectory execution: none; hardware execution: none.
- Joint/model/collision geometry modifications: none.

## Sampling

- Automatically derived conservative bounding box in `base_link`: [-0.38311044767979, -0.606976689406438, 0.210302150595015] to [0.996733816987081, 0.667028617570201, 2.12954389688139] m.
- RobotModel-derived arm reach radius used for the box: 0.623598893884194 m.
- Grid resolution: 7 x 7 x 7
- Voxel size: [0.197120609238124, 0.18200075813952, 0.274177392326625] m; voxel volume: 0.00983641563430309 m^3.
- Physical points: 343; configuration evaluations: 686; configured maximum IK seeds per evaluation: 42.
- Lift candidates (m): 0.05, 0.15, 0.25, 0.35, 0.45, 0.55, 0.65
- Yaw candidates (rad): -0.116355333333333, -2.77555756156289e-17, 0.116355333333333
- Pitch candidates (rad): -0.054541625, 0, 0.185441125, 0.425423875, 0.665406625
- Exact-bound equality epsilon: 1e-08 (numerical equality test only; not a feasibility threshold).

## Results

| Metric | LIFT_ONLY | LIFT_YAW_PITCH |
|---|---:|---:|
| Reachable points | 29 | 35 |
| Estimated workspace volume (m^3) | 0.28525605339479 | 0.344274547200608 |
| X reachable min/max (m) | -0.0874295338226038 / 0.503932293891769 | -0.0874295338226038 / 0.701052903129894 |
| Y reachable min/max (m) | 0.030025964081882 / 0.394027480360922 | 0.030025964081882 / 0.394027480360922 |
| Z reachable min/max (m) | 0.621568239084953 / 1.71827780839145 | 0.621568239084953 / 1.44410041606483 |
| X/Y/Z span (m) | 0.591361827714373 / 0.36400151627904 / 1.0967095693065 | 0.788482436952498 / 0.36400151627904 / 0.822532176979876 |
| Minimum joint margin | 0.00213968741943127 | 1.48199175153962e-08 |
| Minimum self-clearance (m) | 0.000659260018009227 | 0.000659260018009227 |
| Mean manipulability | 0.111343959800405 | 0.326615578594781 |
| Dominant failure reason | NO_IK (314) | NO_IK (308) |

- Common reachable: 28
- Torso-expanded: 7
- Lift-only-only: 1 (reported without suppression)
- Unreachable both: 307
- Volume delta: 0.0590184938058186 m^3
- Percentage volume increase: 20.6896551724138 %
- Largest sampled span increase direction: X (forward/backward) (delta 0.197120609238124 m).
- Torso-expanded selected yaw distribution: -0.116355333333333:2;-2.77555756156289e-17:2;0.116355333333333:3
- Torso-expanded selected pitch distribution: 0.185441125:4;0.425423875:3
- Torso-expanded selected lift distribution: 0.05:1;0.15:2;0.45:2;0.55:2

Jacobian columns mix one prismatic lift coordinate with revolute coordinates; manipulability values are best used comparatively under this identical convention. Boundary points can be identified from successful rows with the smallest joint margin/self-clearance and failed rows classified as self-collision or bounds failures.

## Next-stage guidance

Use `TORSO_EXPANDED` points to place later boxes where waist DOF matter, `COMMON_REACHABLE` interior points for robust baseline tasks, and low-margin/low-clearance reachable neighbors for recovery scenarios. No box placement or task planning was performed in this run.

## Visualization and integrity

- 3D views generated for lift-only, lift+yaw+pitch, and torso-expanded points.
- XY, XZ, and YZ classification projections generated.
- Integrity checks: PASS (row counts, two configurations per point, duplicates, finite tokens, classification consistency, metadata).
- SHA-256 manifest generated and verified after all outputs were finalized.
- Runtime configuration: `/home/openarm/humanoid_sim_ws/install/fixed_base_workspace_analysis/share/fixed_base_workspace_analysis/config/fixed_base_workspace.yaml`

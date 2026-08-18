# FK Workspace Boundary Audit

## Model and scope

- Expanded robot model SHA-256: `af08e6a565f0c5fbf677106fd47176bc699f6b025cccd17f2a10753efa0a19d9`
- Joint position limits source: validated URDF loaded through MoveIt RobotModel; `joint_limits.yaml` supplies velocity policy only.
- Base/TCP: `base_link` / `openarm_left_hand_tcp`.
- Coordinate convention: +X forward, +Y left, +Z vertical.
- AMR/base/world transform: fixed.
- Workspace: positional FK endpoints; TCP orientation is unconstrained.
- IK, OMPL, Cartesian planning, box/object, controller and hardware: not used.

## Sampling and validity

- Method: deterministic 10-dimensional Halton low-discrepancy sequence.
- Seed/index offset: `20260819`.
- Shared dimensions: identical lift and seven arm samples at each sample ID across C0-C3.
- Samples: 10,000 per configuration; 40,000 total; no early stop or automatic rerun.
- Valid state criteria: joint bounds, no exact active bound, existing SRDF ACM self-collision check, finite FK update.
- Boundary reference origin: `base_link` origin.
- Front boundary: Y-Z angle bins; right boundary: X-Z angle bins; 2° bin centers.
- Inner/outer boundaries are observed per-bin radial minima/maxima; convex hull and spline filling are not used.
- Observed gap flag: adjacent sampled radii differ by >0.05 m in a bin with at least 3 samples. It is sampling evidence, not proof of infeasibility.

## Configuration results

| Configuration | Valid / 10000 | Collision rejects | X range (m) | Y range (m) | Z range (m) | Front radius (m) | Right radius (m) |
|---|---:|---:|---:|---:|---:|---:|---:|
| LIFT_ONLY | 7319 | 2681 | -0.176144–0.690382 | -0.094447–0.588702 | 0.581673–1.975077 | 0.608420–1.989876 | 0.638849–2.000948 |
| LIFT_YAW | 7317 | 2683 | -0.200690–0.712144 | -0.105334–0.588425 | 0.581673–1.975077 | 0.607307–1.989945 | 0.637581–2.001377 |
| LIFT_PITCH | 4332 | 5668 | -0.176427–0.750428 | -0.094447–0.588702 | 0.566327–1.952503 | 0.581675–1.965784 | 0.623993–1.986237 |
| LIFT_YAW_PITCH | 4961 | 5039 | -0.193802–0.788695 | -0.104869–0.602058 | 0.561256–1.952503 | 0.581673–1.965187 | 0.576969–1.986787 |

## Observed radial gaps

| Configuration | Front bins | Front observed gaps | Right bins | Right observed gaps |
|---|---:|---:|---:|---:|
| LIFT_ONLY | 20 | 26 | 26 | 33 |
| LIFT_YAW | 20 | 23 | 26 | 37 |
| LIFT_PITCH | 20 | 35 | 27 | 53 |
| LIFT_YAW_PITCH | 22 | 36 | 29 | 58 |

## Convergence observations (7,500 → 10,000 samples)

| Configuration | max X (m) | Δ max X (m) | Front observed band area (m²) | Right observed band area (m²) |
|---|---:|---:|---:|---:|
| LIFT_ONLY | 0.690382 → 0.690382 | +0.000000 | 0.800676 → 0.811889 | 0.986888 → 1.008647 |
| LIFT_YAW | 0.709097 → 0.712144 | +0.003047 | 0.808283 → 0.825577 | 1.009634 → 1.030621 |
| LIFT_PITCH | 0.750428 → 0.750428 | +0.000000 | 0.778300 → 0.797369 | 0.969834 → 1.006275 |
| LIFT_YAW_PITCH | 0.762769 → 0.788695 | +0.025926 | 0.792222 → 0.822339 | 1.008781 → 1.047551 |

No arbitrary early-stop threshold was applied; all 10,000 states per configuration were evaluated.

## FK versus existing fixed-orientation targeted grid

| Configuration | FK max X | Grid max X | ΔX | FK Y span | Grid Y span | FK Z span | Grid Z span | Interpretation |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| LIFT_ONLY | 0.690382 | 0.677083 | 0.013299 | 0.683149 | 0.378000 | 1.393403 | 1.054167 | MAX_X_WITHIN_ONE_GRID_STEP |
| LIFT_YAW | 0.712144 | 0.706250 | 0.005894 | 0.693759 | 0.378000 | 1.393403 | 1.054167 | MAX_X_WITHIN_ONE_GRID_STEP |
| LIFT_PITCH | 0.750428 | 0.706250 | 0.044178 | 0.683149 | 0.378000 | 1.386176 | 1.054167 | POSITIONAL_FK_EXTENDS_BEYOND_FIXED_ORIENTATION_GRID |
| LIFT_YAW_PITCH | 0.788695 | 0.735417 | 0.053278 | 0.706928 | 0.378000 | 1.391247 | 1.054167 | POSITIONAL_FK_EXTENDS_BEYOND_FIXED_ORIENTATION_GRID |

The FK set is orientation-unconstrained and samples joint space, while the grid set uses a fixed grasp orientation inside a targeted Cartesian box. Differences therefore quantify method/scope, not a contradiction or replacement.

## Presentation and integrity

- Figures: 10 PNG, 1920×1080, with raw endpoint scatter plus observed inner/outer boundaries.
- Video: `presentation/fk_workspace_boundary_demo.mp4` (35.0 s, 1920x1080, 30 fps, H.264, 3910172 bytes); full 1050-frame decode PASS.
- Existing seven workspace/demo manifests: PASS before and after analysis.
- Protected Xacro/URDF, SRDF/ACM, joint limits, kinematics and OMPL hashes: unchanged.
- Actual trajectory execution: false; controller: false; ros2_control: false; hardware: false; AMR motion: false.

# Workspace Front/Right Projection Audit

## Source and coordinate frame

- Source CSV: `validation/fixed_base_workspace_dof_ablation_comparison.csv`
- Source SHA-256: `5d7f1ea256b99ee0a36b1e8eab9250b34d89bcf9dce3c9fc61671eccd10f96b2`
- Source physical TCP points: **1,440 unique points**
- Source reachable counts C0/C1/C2/C3: **[833, 1030, 976, 1119]**
- Base frame: `base_link`
- Verified convention: **+X forward, +Y left, +Z up**
- Front view: **Y-Z plane**, viewed from the robot front; X is collapsed.
- Right-side view: **X-Z plane**, viewed from the robot right; Y is collapsed.

## Projection method

- Definition: **2D union projection of validated 3D reachable workspace**.
- Front cell is reachable if any validated X-depth sample at that (Y,Z) is reachable.
- Right cell is reachable if any validated Y-depth sample at that (X,Z) is reachable.
- This union does not imply that every collapsed-depth coordinate is reachable.
- Convex hull: **not used**.
- Smoothing/interpolation/morphological closing: **not used**.
- Unreachable 2D holes: **preserved exactly at sampled grid resolution**.
- New IK/workspace sampling: **none**.

## Grid and projected cell areas

- Grid dimensions X/Y/Z: **12 / 10 / 12**
- dx/dy/dz: **0.029166666666667 / 0.042000000000000 / 0.095833333333333 m**
- Front Y-Z cell area: **0.004025000000000 m²**
- Right X-Z cell area: **0.002795138888889 m²**

## Results

| Configuration | Front cells | Front area (m²) | Δ front vs C0 | Right cells | Right area (m²) | Δ right vs C0 | Max X (m) |
|---|---:|---:|---:|---:|---:|---:|---:|
| LIFT_ONLY | 109 | 0.438725 | +0.00% | 103 | 0.287899 | +0.00% | 0.677083 |
| LIFT_YAW | 118 | 0.474950 | +8.26% | 116 | 0.324236 | +12.62% | 0.706250 |
| LIFT_PITCH | 113 | 0.454825 | +3.67% | 116 | 0.324236 | +12.62% | 0.706250 |
| LIFT_YAW_PITCH | 119 | 0.478975 | +9.17% | 125 | 0.349392 | +21.36% | 0.735417 |

The projected maximum X values exactly match the validated C0-C3 maxima (0.677083, 0.706250, 0.706250, 0.735417 m).

## Presentation and RViz

- Figures: **18 PNG files, each 1920×1080**, including all requested 12 primary figures.
- RViz scenes: `front_c0`, `front_c1`, `front_c2`, `front_c3`, `front_compare`, `right_c0`, `right_c1`, `right_c2`, `right_c3`, `right_compare`.
- RViz uses the same generated projection CSV cells and the fixed `base_link` frame.
- Video: `presentation/workspace_front_right_projection_demo.mp4` (36.0 s, 1920x1080, 30 fps, H.264, 3212288 bytes).
- Full video decode: **PASS (1080 frames)**.

## Safety scope

- AMR/base fixed: **true**
- Actual trajectory execution: **false**
- Controller: **false**
- ros2_control: **false**
- Hardware/motor/sensor use: **false**
- Existing coarse/fine/ablation/envelope/radial manifests: **PASS and unchanged**

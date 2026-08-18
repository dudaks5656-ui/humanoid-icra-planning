# Directed Boundary-Sweep Differential Visualization Audit

## Immutable sources

- Source commit before this visualization: `3d35580d8b4e764374e4c030a33a0f2a285183d3`.
- Existing boundary-sweep manifest: PASS (43 entries).
- `validation/boundary_sweep_workspace_nested_states.csv`: `3a5e49c62dce5858c3fa18157fb4d39a72a3ba02a8740aaf47e4679bb9382221`
- `validation/boundary_sweep_workspace_nested_check.csv`: `b8796eaafd9156f9f3a3dbfbb79eacba86c5e1daffe706b680db4d3a136845a7`
- `validation/boundary_sweep_workspace_3d_surface.csv`: `237867fa065ea464e77d6de0dd53e9c191cbd0c2bf9bd0df34500859556704bc`
- `validation/boundary_sweep_workspace_summary.csv`: `e496c5c905d09497364b716ac23574035de670233bbca525b58fba9aeef36292`
- `validation/boundary_sweep_workspace_manifest_sha256.txt`: `81c3fb8a932f4113cb66264312deabff8eedddb451d3be4e694ac84d7e19f12c`

No URDF, SRDF/ACM, joint limits, directed-sweep CSV, existing 3D shell, existing manifest, or existing video was modified.

## Five-way decomposition

- BASELINE_C0: no support-metric improvement over C0.
- YAW_UNIQUE: C1 improves C0 and C2 does not.
- PITCH_UNIQUE: C2 improves C0 and C1 does not.
- SINGLE_DOF_SHARED: C1 and C2 both independently improve C0.
- COMBINED_ONLY: C3 improves beyond both C1 and C2.
- Classification uses the validated `(view, boundary, Lift slice, sweep angle)` correspondence. Triangle metrics are the mean of their three endpoint metrics.
- This is a disjoint directed-support surface-patch decomposition, not a volumetric mesh Boolean subtraction.

| Category | Endpoints | Patches | Approx. patch area (m²) |
|---|---:|---:|---:|
| BASELINE_C0 | 87 | 53 | 0.300694 |
| YAW_UNIQUE | 114 | 149 | 0.596129 |
| PITCH_UNIQUE | 57 | 65 | 0.445814 |
| SINGLE_DOF_SHARED | 54 | 72 | 0.271994 |
| COMBINED_ONLY | 168 | 429 | 3.528609 |

## Identity and nested validity

- `C3 = BASELINE_C0 ∪ YAW_UNIQUE ∪ PITCH_UNIQUE ∪ SINGLE_DOF_SHARED ∪ COMBINED_ONLY`: PASS.
- 768/768 selected C3 patches are assigned once; category patch-ID intersection is empty.
- 480/480 correspondence endpoints are assigned once.
- Existing nested relations C0⊆C1/C2 and C1/C2⊆C3: 20/20 PASS.
- C0 max X = 0.6084 m; C3 max X = 0.7681 m; increase = +0.1596 m (+26.2%).

## Presentation

- `presentation/boundary_3d_differential.png`
- `presentation/boundary_3d_four_configurations.png`
- `presentation/boundary_3d_c0_vs_c3_expansion.png`
- `presentation/boundary_3d_yaw_effect.png`
- `presentation/boundary_3d_pitch_effect.png`
- `presentation/boundary_3d_single_dof_shared.png`
- `presentation/boundary_3d_combined_only.png`
- All figures use the same forward-emphasizing camera, base reference, axis limits and scale where compared.
- Legends are outside the plotting axes and are not clipped.
- The four complete shells are shown only in separate 2×2 panels, never as the main overlay.

## RViz

- Scenes: `boundary_diff_all`, `boundary_diff_yaw`, `boundary_diff_pitch`, `boundary_diff_combined`, `boundary_c0_vs_c3`.
- Marker namespaces: `baseline_c0`, `yaw_unique`, `pitch_unique`, `single_dof_shared`, `combined_only`.
- RobotState publication is visualization-only; no trajectory execution or controller is used.

## Video

- `presentation/boundary_sweep_workspace_differential_demo.mp4`: 32.0 s, 1920x1080, 30/1 fps, H.264 High level 4.0, yuv420p, 837034 bytes.
- Full decode: 960/960 frames PASS; audio absent; faststart `moov` offset 36 before `mdat` offset 12352.
- Sequence adds one contribution at a time; four complete shells are not overlaid.

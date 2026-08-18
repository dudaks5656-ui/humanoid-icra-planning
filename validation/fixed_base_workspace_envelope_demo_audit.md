# Fixed-base workspace envelope demo audit

## Immutable sources

- `/home/openarm/humanoid_sim_ws/validation/fixed_base_workspace_dof_ablation_comparison.csv` — `5d7f1ea256b99ee0a36b1e8eab9250b34d89bcf9dce3c9fc61671eccd10f96b2`
- `/home/openarm/humanoid_sim_ws/validation/fixed_base_workspace_dof_ablation_summary.csv` — `f4c35452f7d896831931441329f13b5fdd680c423d5f7648ec45784d97b6c330`
- `/home/openarm/humanoid_sim_ws/validation/fixed_base_workspace_dof_ablation_contributions.csv` — `e17a42791e5c384b686edc9bb2f254a20241ec6f1192260e937637ef4ae61a0c`
- Existing coarse/fine/ablation/demo manifests verified before implementation: PASS

## Grid and reconstruction

- Grid dimensions: `12 × 10 × 12` = 1,440 physical TCP points
- Spacing: dx `0.029166666666667` m, dy `0.042` m, dz `0.095833333333333` m
- Voxel volume: `0.000117395833333334` m³
- Method: occupied voxel + six-neighbor exposed-face extraction
- Convex hull: not used
- Hole preservation: yes; no filling or smoothing

| Region | Occupied | Exposed faces | Triangles | Reconstructed m³ | Validated m³ | Difference |
|---|---:|---:|---:|---:|---:|---:|
| C0 | 833 | 636 | 1272 | 0.097790729166667 | 0.097790729166666 | 8.743e-16 |
| C1 | 1030 | 674 | 1348 | 0.120917708333334 | 0.120917708333333 | 1.082e-15 |
| C2 | 976 | 694 | 1388 | 0.114578333333334 | 0.114578333333333 | 1.041e-15 |
| C3 | 1119 | 730 | 1460 | 0.131365937500001 | 0.131365937500000 | 8.049e-16 |
| COMBINED_ONLY | 65 | 266 | 532 | 0.007630729166667 | 0.007630729166667 | 6.332e-17 |

## Representative RobotState visualization

- Boundary point sets: C0=[1096;110;11;72;534]; C1=[1218;109;11;0;534]; C2=[1215;110;11;60;534]; C3=[1360;1340;109;11;0;654]
- Animated point states: C0=1096 (true); C1=1218 (true); C2=1215 (true); C3=1360 (true)
- Combined-only representative: point `1360`, TCP `(0.735416666666667, 0.147, 1.03125)` m
- Point 1360 classification: C0/C1/C2 FAIL, C3 PASS
- Interpolation policy: neutral-to-valid RobotState; every intermediate sample checked for bounds and self-collision
- Failed interpolation is not displayed as motion; static state/markers are used instead

## Recording

- Full: `/home/openarm/humanoid_sim_ws/presentation/fixed_base_workspace_envelope_demo.mp4`, 65.934 s, 1920×1080, 30.0 fps, H.264/MP4, 10034990 bytes, all frames decoded `True`
- Short: `/home/openarm/humanoid_sim_ws/presentation/fixed_base_workspace_envelope_demo_short.mp4`, 42.938 s, 1920×1080, 30.0 fps, H.264/MP4, 6362419 bytes, all frames decoded `True`
- Screenshots: `envelope_c0.png`, `envelope_c1.png`, `envelope_c2.png`, `envelope_c3.png`, `envelope_c0_vs_c3.png`, `envelope_combined_only.png`

## Safety

- AMR/base fixed: true
- trajectory execution: false
- controller: false
- ros2_control: false
- hardware: false
- navigation: false

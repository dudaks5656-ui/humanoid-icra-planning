# Targeted fine fixed-base workspace audit

Generated: 2026-08-18T12:50:34Z

## Scope and sampling

- Existing coarse manifest pre-check and post-check: **PASS**. Coarse files were not modified.
- Fine targeted bounding box in `base_link` (+X forward, +Y left, +Z up): X [0.4, 0.75], Y [0, 0.42], Z [0.6, 1.75] m.
- Grid: 12 x 10 x 12 = 1440 physical points; 2880 main configuration evaluations.
- Voxel size: [0.0291666666666667, 0.042, 0.0958333333333333] m; volume 0.000117395833333333 m^3.
- MAX_IK_SEEDS: 150; special validation: 300; fixed random seed: 20260818.
- Seed strategy: `NEIGHBOR_THEN_COARSE_TORSO_THEN_DEFAULT_HALTON_FIXED_RANDOM`; identical policy for both configurations.
- Confidence: HIGH requires repeated support plus at least two same-class axial neighbors; MEDIUM requires repeated support or a valid solution; otherwise LOW. Confidence never changes feasibility.
- TCP orientation remains KINEMATIC_REFERENCE_GRASP_50MM, xyzw [0, 1, 0, 0], tolerance 0.03 rad.
- Environment/box/target objects: none. Base fixed; trajectory execution/controller/ros2_control/hardware: none.

## Fine targeted results

| Metric | LIFT_ONLY | LIFT_YAW_PITCH |
|---|---:|---:|
| Reachable points | 833 | 1092 |
| Targeted volume (m^3) | 0.0977907291666666 | 0.12819625 |
| Maximum forward X (m) | 0.677083333333333 | 0.735416666666667 |
| Mean joint margin | 0.131780092686084 | 0.104322528303093 |
| Minimum joint margin | 3.94734773756511e-08 | 1.47019143525995e-08 |
| Mean self-clearance (m) | 0.000659260018009213 | 0.000658671661514745 |
| Minimum self-clearance (m) | 0.000659260018009227 | 1.67747260502566e-05 |
| Mean manipulability | 0.0948813878854551 | 0.359910993396048 |

- Forward reach delta: 0.0583333333333333 m.
- COMMON_REACHABLE: 833; TORSO_EXPANDED: 259; LIFT_ONLY_ONLY: 0; UNREACHABLE_BOTH: 348.
- Targeted-region delta volume: 0.0304055208333333 m^3 (31.0924%).
- Torso-expanded voxel volume: 0.0304055208333333 m^3. This is not a whole-workspace volume.
- The coarse whole-box +20.6897% figure and this targeted percentage have different integration domains; only direction/trend is compared.

## Coarse anomaly and high-Z validation

- Exact coarse anomaly (0.503932293891769, 0.212026722221402, 1.71827780839145): A=1, B=1, verdict **SAMPLING_ARTIFACT**, seeds A/B=241/263.
- 3x3x3 anomaly neighborhood evaluated: 27 physical points, both configurations, up to 300 seeds.
- High-Z A-pass/B-fail points receiving 300-seed B validation: 0; recovered for B: 0.
- Coarse/fine nearest-voxel consistency: {'COARSE_FAIL_FINE_FAIL': 31, 'COARSE_PASS_FINE_PASS': 25, 'COARSE_FAIL_FINE_PASS': 4}. COARSE_PASS_FINE_FAIL rows retain their fine failure reason.

## Torso use and box-position candidates

- Torso-expanded selected yaw distribution: -0.0436332312998582:37;-0.116355333333333:29;-2.77555756156289e-17:10;0:65;0.116355333333333:118
- Torso-expanded selected pitch distribution: -0.0436332312998582:3;-0.0872664625997165:1;-0.130899693899575:1;0:36;0.0436332312998582:3;0.0872664625997165:11;0.130899693899575:48;0.185441125:39;0.425423875:117
- Torso-expanded selected lift distribution: 0.05:17;0.15:28;0.25:10;0.35:12;0.4:2;0.45:26;0.55:164
- Candidate rows: stable common 10, workspace boundary 10, torso-dependent 10.
- Recommended STABLE_COMMON_REGION: (0.414583333333333, 0.147, 1.22291666666667) m — ranked by interior distance=0.2912, conservative margin/clearance/manipulability.
- Recommended WORKSPACE_BOUNDARY_REGION: (0.647916666666667, 0.189, 0.839583333333333) m — 4 adjacent voxels are non-common; near sampled forward edge.
- Recommended TORSO_DEPENDENT_REGION: (0.677083333333333, 0.063, 1.31875) m — Lift-only exhausted search while torso configuration passed.

No box collision, grasp path, attachment, recovery planning, AMR motion, or subsequent experiment was started.

## Integrity and visualization

- Visualization: PASS.
- Fine row-count, duplicate, finite-value, two-configuration, classification, expanded-table, boundary-table, and coarse-consistency checks: PASS.
- Coarse manifest entries verified after fine processing: 14.
- Fine SHA-256 manifest generated and verified after all outputs were finalized.

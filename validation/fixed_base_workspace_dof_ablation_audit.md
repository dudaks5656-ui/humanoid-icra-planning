# Fixed-base waist DOF ablation audit

Generated: 2026-08-18T13:46:00Z

## Reuse contract and scope

- Coarse and fine manifests verified before and after: **PASS**; all prior evidence remains unchanged.
- C0 LIFT_ONLY and C3 LIFT_YAW_PITCH are reused from the immutable fine CSV; neither was globally re-executed.
- C1 LIFT_YAW and C2 LIFT_PITCH each evaluated the exact same 1,440 point IDs and XYZ coordinates.
- Robot/Xacro SHA-256: `e0bcfb64859d94203eeec45f823dec8d238d72bec6809d81c2c99091942be4e4`; SRDF/ACM SHA-256: `7892b122fe28e91650f8919a66c256ebc55d3b34472e006701a63cadab5484e8`.
- Joint-limits SHA-256: `1e851fa791dab1c95eeba4a698e620c0334afd12a0c9964b774f24868d03ccfb`; kinematics SHA-256: `7a825c85e6d3bfa06b3ff6d195b3aced8c490d6132d8d94e2d0b01f52e73204e`.
- Orientation xyzw [0,1,0,0], tolerance 0.03 rad; inherited fine validity and seed policy, MAX_IK_SEEDS=150, fixed random seed 20260818.
- C1 pitch was reasserted to zero and C2 yaw was reasserted to zero before and after every arm IK call.
- Base fixed; no environment/box/target, trajectory execution, AMR motion, controller, ros2_control, or hardware.

## Four-way targeted workspace

| Configuration | Reachable | Volume (m^3) | Delta vs C0 | Delta % | X/Y/Z max (m) |
|---|---:|---:|---:|---:|---|
| LIFT_ONLY | 833 | 0.0977907291666664 | 0 | 0 | 0.677083333333333 / 0.399 / 1.70208333333333 |
| LIFT_YAW | 1030 | 0.120917708333333 | 0.0231269791666666 | 23.6494597839136 | 0.70625 / 0.399 / 1.70208333333333 |
| LIFT_PITCH | 976 | 0.114578333333333 | 0.0167876041666666 | 17.1668667466987 | 0.70625 / 0.399 / 1.70208333333333 |
| LIFT_YAW_PITCH | 1119 | 0.1313659375 | 0.0335752083333332 | 34.3337334933974 | 0.735416666666667 / 0.399 / 1.70208333333333 |

- Yaw unique newly reachable: 78 points.
- Pitch unique newly reachable: 24 points.
- Yaw/Pitch single-DOF overlap: 119 points.
- COMBINED_TORSO_ONLY: 65 points, 0.00763072916666665 m^3.
- Combined extra gain V3-max(V1,V2): 0.010448229166667 m^3. This is a set-volume difference, not an additive-synergy claim.
- Dominant span increase: Yaw=X {'X': 0.029166666666667007, 'Y': 0.0, 'Z': 0.0}, Pitch=X {'X': 0.029166666666667007, 'Y': 0.0, 'Z': 0.0}, Full=X {'X': 0.058333333333334014, 'Y': 0.0, 'Z': 0.0}.

## Common-point posture quality

All-four-pass common points: 833. Means/medians below use this identical point set.

| Configuration | Manipulability mean/median | Joint margin mean/median | Active revolute margin mean/median | Self-clearance mean/median (m) |
|---|---:|---:|---:|---:|
| LIFT_ONLY | 0.0948813878854552 / 0.0918594776451555 | 0.131780092686084 / 0.15 | 0.196238315673955 / 0.184285011043057 | 0.000659260018009228 / 0.000659260018009228 |
| LIFT_YAW | 0.204453686448807 / 0.198791975483807 | 0.112972080481352 / 0.149967605318748 | 0.122540721922724 / 0.162452531634308 | 0.000659260018009229 / 0.000659260018009228 |
| LIFT_PITCH | 0.220089463095588 / 0.219612073042972 | 0.128606104030102 / 0.15 | 0.163230128965852 / 0.174533 | 0.000659260018009229 / 0.000659260018009228 |
| LIFT_YAW_PITCH | 0.357221511592261 / 0.359790028638344 | 0.120131078523778 / 0.15 | 0.137861939560595 / 0.174533 | 0.000659260018009229 / 0.000659260018009228 |

Jacobian column sets differ by active DOF, so manipulability is interpreted comparatively with that configuration convention and common XYZ support.

## Sampling consistency and torso use

- Nested anomaly rows: 28; verdicts: {'SAMPLING_ARTIFACT': 28}.
- Final classification counts: {'YAW_EXPANDED': 78, 'BASELINE_REACHABLE': 833, 'YAW_AND_PITCH_INDIVIDUALLY_CAPABLE': 119, 'UNREACHABLE_ALL': 321, 'COMBINED_TORSO_ONLY': 65, 'PITCH_EXPANDED': 24}.
- LIFT_PITCH selected pitch raw distribution: -0.174532:4;-0.130899693899575:2;-0.0872664625997165:5;-0.0436332312998582:79;0:431;0.0436332312998582:48;0.0872664625997165:96;0.130899693899575:237;0.174532925199433:74
- LIFT_YAW selected yaw raw distribution: -0.174532:43;-0.130899693899575:193;-0.0872664625997165:93;-0.0436332312998582:72;0:625;0.0436332312998582:1;0.0872664625997165:3
- LIFT_YAW_PITCH selected pitch raw distribution: -0.130899693899575:1;-0.0872664625997165:11;-0.0436332312998582:95;0:481;0.0436332312998582:15;0.0872664625997165:17;0.130899693899575:264;0.174532925199433:8;0.185441125:88;0.425423875:139
- LIFT_YAW_PITCH selected yaw raw distribution: -0.174532:4;-0.130899693899575:8;-0.116355333333333:42;-0.0872664625997165:3;-0.0436332312998582:58;-2.77555756156289e-17:46;0:808;0.116355333333333:150

## Integrity and stopping condition

- Visualization: PASS.
- C1/C2 counts, four-way coordinates, duplicates, finite tokens, classifications, boundaries, common metrics, and nested checks: PASS.
- Coarse manifest entries verified: 14; fine manifest entries verified: 21.
- Ablation SHA-256 manifest generated and verified. No box or recovery experiment was started.

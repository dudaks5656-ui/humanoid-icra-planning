# Lift-Slice FK Workspace Audit

## Model and scope

- Expanded robot model SHA-256: `af08e6a565f0c5fbf677106fd47176bc699f6b025cccd17f2a10753efa0a19d9`.
- Base/TCP/reference: `base_link` / `openarm_left_hand_tcp` / `openarm_left_link0`.
- Coordinate convention: +X forward, +Y left, +Z vertical; AMR/base fixed.
- Main result: orientation-unconstrained POSITIONAL_FK_WORKSPACE.
- Sampling: deterministic 9-dimensional Halton sequence, seed/index offset `20260819`.
- 2,000 attempts × 4 configurations × 5 lift slices = 40,000 attempts; no rerun/refinement.
- Validity: actual RobotModel joint bounds, exact sampled-active bound exclusion, SRDF ACM self-collision check, finite FK.
- Lift MIN/MAX are deliberately fixed slice contexts and exempt from active exact-bound rejection; sampled Arm/Yaw/Pitch remain checked.
- Lift axis semantics: q=MIN is physical topmost and increasing q translates the lift along ROS -Z.
- IK, OMPL, trajectory execution, controller, ros2_control, hardware and AMR motion: not used.

## Lift slices

- Actual limit: `0.000000` to `0.700000` m.
- MIN / 25% / MID / 75% / MAX: `0.000000, 0.175000, 0.350000, 0.525000, 0.700000` m.
- Radial reference is the canonical shoulder link `openarm_left_link0` at the relevant lift slice, yaw=0, pitch=0 and default arm.

## Per-slice results

| Config | Lift % | Own valid | Nested pool | X range | Y range | Z range | Front r inner–outer | Right r inner–outer | Collision rejects |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| LIFT_ONLY | 0 | 1904 | 1904 | -0.178–0.686 | -0.077–0.588 | 1.122–1.989 | 0.008–0.558 | 0.006–0.436 | 96 |
| LIFT_ONLY | 25 | 1856 | 1856 | -0.178–0.686 | -0.077–0.588 | 0.947–1.814 | 0.008–0.558 | 0.006–0.436 | 144 |
| LIFT_ONLY | 50 | 1805 | 1805 | -0.163–0.686 | -0.077–0.588 | 0.772–1.639 | 0.008–0.558 | 0.006–0.436 | 195 |
| LIFT_ONLY | 75 | 1764 | 1764 | -0.163–0.686 | -0.077–0.588 | 0.597–1.464 | 0.008–0.558 | 0.006–0.436 | 236 |
| LIFT_ONLY | 100 | 0 | 0 | NO_VALID_STATE–NO_VALID_STATE | NO_VALID_STATE–NO_VALID_STATE | NO_VALID_STATE–NO_VALID_STATE | NO_VALID_STATE–NO_VALID_STATE | NO_VALID_STATE–NO_VALID_STATE | 2000 |
| LIFT_YAW | 0 | 1903 | 3807 | -0.197–0.707 | -0.090–0.588 | 1.122–1.989 | 0.006–0.558 | 0.005–0.456 | 97 |
| LIFT_YAW | 25 | 1851 | 3707 | -0.197–0.707 | -0.090–0.588 | 0.947–1.814 | 0.006–0.558 | 0.005–0.456 | 149 |
| LIFT_YAW | 50 | 1794 | 3599 | -0.163–0.707 | -0.090–0.588 | 0.772–1.639 | 0.006–0.558 | 0.005–0.456 | 206 |
| LIFT_YAW | 75 | 1758 | 3522 | -0.163–0.707 | -0.090–0.588 | 0.597–1.464 | 0.006–0.558 | 0.005–0.456 | 242 |
| LIFT_YAW | 100 | 0 | 0 | NO_VALID_STATE–NO_VALID_STATE | NO_VALID_STATE–NO_VALID_STATE | NO_VALID_STATE–NO_VALID_STATE | NO_VALID_STATE–NO_VALID_STATE | NO_VALID_STATE–NO_VALID_STATE | 2000 |
| LIFT_PITCH | 0 | 1126 | 3030 | -0.178–0.752 | -0.077–0.588 | 1.091–1.990 | 0.008–0.561 | 0.003–0.507 | 874 |
| LIFT_PITCH | 25 | 1101 | 2957 | -0.178–0.752 | -0.077–0.588 | 0.916–1.815 | 0.008–0.561 | 0.003–0.507 | 899 |
| LIFT_PITCH | 50 | 1074 | 2879 | -0.176–0.752 | -0.077–0.588 | 0.741–1.640 | 0.008–0.561 | 0.003–0.507 | 926 |
| LIFT_PITCH | 75 | 1053 | 2817 | -0.176–0.752 | -0.077–0.588 | 0.566–1.465 | 0.008–0.561 | 0.003–0.507 | 947 |
| LIFT_PITCH | 100 | 0 | 0 | NO_VALID_STATE–NO_VALID_STATE | NO_VALID_STATE–NO_VALID_STATE | NO_VALID_STATE–NO_VALID_STATE | NO_VALID_STATE–NO_VALID_STATE | NO_VALID_STATE–NO_VALID_STATE | 2000 |
| LIFT_YAW_PITCH | 0 | 1276 | 6209 | -0.197–0.754 | -0.092–0.602 | 1.091–1.990 | 0.006–0.584 | 0.003–0.509 | 724 |
| LIFT_YAW_PITCH | 25 | 1244 | 6052 | -0.197–0.754 | -0.092–0.602 | 0.916–1.815 | 0.006–0.584 | 0.003–0.509 | 756 |
| LIFT_YAW_PITCH | 50 | 1213 | 5886 | -0.188–0.754 | -0.090–0.602 | 0.741–1.640 | 0.006–0.584 | 0.003–0.509 | 787 |
| LIFT_YAW_PITCH | 75 | 1191 | 5766 | -0.188–0.754 | -0.090–0.602 | 0.566–1.465 | 0.006–0.584 | 0.003–0.509 | 809 |
| LIFT_YAW_PITCH | 100 | 0 | 0 | NO_VALID_STATE–NO_VALID_STATE | NO_VALID_STATE–NO_VALID_STATE | NO_VALID_STATE–NO_VALID_STATE | NO_VALID_STATE–NO_VALID_STATE | NO_VALID_STATE–NO_VALID_STATE | 2000 |

## Nested inclusion

- State pools are structural unions: C1=C0∪C1-own, C2=C0∪C2-own, C3=C0∪C1-own∪C2-own∪C3-own.
- Checks: 20 / 20 PASS; missing inherited endpoint states: 0.
- Therefore C0⊆C1, C0⊆C2, C1⊆C3 and C2⊆C3 at every lift slice.

## Boundary, gaps, and 3D loft

- Front is Y-Z; right is X-Z. Inner/outer are observed radial extrema in 2° bins about the slice-specific shoulder reference.
- Observed gaps (>0.05 m adjacent radial sample separation, ≥3 samples/bin): front 2176, right 4252. These are observations, not infeasibility proofs.
- No convex hull, spline filling, or morphological closing is used.
- 3D loft has 980 triangles. It joins corresponding non-gapped outer spherical bins across adjacent lift slices; missing/gapped bins remain open.
- Loft is a linear five-slice approximation, not a continuously proven swept volume.

### Adjacent-slice boundary displacement

| Config | Slice pair | Matched bins | Mean displacement | Max displacement |
|---|---|---:|---:|---:|
| LIFT_ONLY | 0→25% | 101 | 0.1750 | 0.1750 |
| LIFT_ONLY | 25→50% | 100 | 0.1750 | 0.1750 |
| LIFT_ONLY | 50→75% | 98 | 0.1768 | 0.3527 |
| LIFT_ONLY | 75→100% | 0 | 0.0000 | 0.0000 |
| LIFT_YAW | 0→25% | 90 | 0.1750 | 0.1750 |
| LIFT_YAW | 25→50% | 91 | 0.1758 | 0.2305 |
| LIFT_YAW | 50→75% | 88 | 0.1766 | 0.3132 |
| LIFT_YAW | 75→100% | 0 | 0.0000 | 0.0000 |
| LIFT_PITCH | 0→25% | 104 | 0.1750 | 0.1750 |
| LIFT_PITCH | 25→50% | 107 | 0.1750 | 0.1750 |
| LIFT_PITCH | 50→75% | 107 | 0.1786 | 0.3527 |
| LIFT_PITCH | 75→100% | 0 | 0.0000 | 0.0000 |
| LIFT_YAW_PITCH | 0→25% | 101 | 0.1750 | 0.1750 |
| LIFT_YAW_PITCH | 25→50% | 106 | 0.1751 | 0.1898 |
| LIFT_YAW_PITCH | 50→75% | 105 | 0.1769 | 0.3132 |
| LIFT_YAW_PITCH | 75→100% | 0 | 0.0000 | 0.0000 |

## Existing fixed-orientation grid cross-check

| Config | FK max X | Grid max X | ΔX | FK Y span | Grid Y span | FK Z span | Grid Z span |
|---|---:|---:|---:|---:|---:|---:|---:|
| LIFT_ONLY | 0.6861 | 0.6771 | +0.0090 | 0.6652 | 0.3780 | 1.3923 | 1.0542 |
| LIFT_YAW | 0.7069 | 0.7063 | +0.0007 | 0.6782 | 0.3780 | 1.3923 | 1.0542 |
| LIFT_PITCH | 0.7523 | 0.7063 | +0.0461 | 0.6652 | 0.3780 | 1.4248 | 1.0542 |
| LIFT_YAW_PITCH | 0.7537 | 0.7354 | +0.0183 | 0.6940 | 0.3780 | 1.4248 | 1.0542 |

The FK workspace is joint-space sampled and orientation-free, so it is expected to exceed the targeted fixed-grasp-orientation Cartesian grid; it cross-validates configuration trends but does not replace the grid result.

## Presentation and integrity

- Figures: 15 PNG; external legends used for 2D figures and comparison figures. Minimum image size: 1920×1080 px.
- Video: `presentation/lift_slice_fk_workspace_demo.mp4`, 45.0 s, 1920x1080, H.264, 8775220 bytes; full 1350-frame decode PASS.
- Existing manifests verified: fixed_base_workspace_manifest_sha256.txt, fixed_base_workspace_fine_manifest_sha256.txt, fixed_base_workspace_dof_ablation_manifest_sha256.txt, fixed_base_workspace_demo_manifest_sha256.txt, fixed_base_workspace_envelope_demo_manifest_sha256.txt, radial_workspace_validation_manifest_sha256.txt, workspace_projection_manifest_sha256.txt, fk_workspace_boundary_manifest_sha256.txt.
- Protected Xacro/URDF, SRDF/ACM, joint limits, kinematics and OMPL hashes unchanged.
- Legend placement: outside plotting area with a reserved fixed-canvas margin; 1920×1080 output verified without clipping.

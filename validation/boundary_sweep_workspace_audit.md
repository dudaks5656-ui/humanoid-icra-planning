# Directed Boundary-Sweep Workspace Audit

## Model and scope

- Expanded RobotModel SHA-256: `af08e6a565f0c5fbf677106fd47176bc699f6b025cccd17f2a10753efa0a19d9`.
- Frames: base `base_link`, TCP `openarm_left_hand_tcp`, radial reference `openarm_left_link0`.
- Coordinates: +X forward, +Y left, +Z up; AMR/base fixed.
- Method: directed FK boundary optimization, not joint-space random workspace sampling.
- No IK, OMPL, Cartesian grid, task planning, trajectory execution, controller, ros2_control or hardware.

## Usable Lift range

- URDF limit: 0.000–0.700 m; collision-free operational range from scan: `0–0.54 m` (bottom–top).
- Scan step: 0.020 m; eight deterministic representative arm postures per position.
- At 0.000–0.200 m: 8/8 representatives valid; 0.220–0.540 m: 7/8 valid; 0.560–0.700 m: 0/8 valid and 8/8 self-colliding.
- Boundary slices: 0.000, 0.135, 0.270, 0.405, 0.540 m.

## Directed sweep

- Each view uses 24 support directions at 15° spacing.
- For each direction: one deterministic valid seed; joint-by-joint global coarse fractions 10/30/50/70/90%, then local ±5° and ±1° refinement.
- Front Y-Z outer objective maximizes directional support; right X-Z does the same. Inner objective minimizes shoulder-relative radius with angular mismatch penalty.
- Collision checks: 140520; self-collision rejects: 8141; valid evaluations: 129678.
- Every emitted boundary state satisfies joint bounds, avoids exact sampled-active bounds, and passes existing SRDF ACM self-collision checking.

## Results

| Config | Front min–max r (m) | Right min–max r (m) | Max X (m) | Max |Y| (m) | Front area (m²) | Right area (m²) |
|---|---:|---:|---:|---:|---:|---:|
| LIFT_ONLY | 0.0138–0.5572 | 0.0301–0.4322 | 0.6084 | 0.5866 | 0.6864 | 0.6483 |
| LIFT_YAW | 0.0065–0.5572 | 0.0301–0.4350 | 0.6615 | 0.5866 | 0.7303 | 0.7718 |
| LIFT_PITCH | 0.0138–0.5572 | 0.0156–0.4999 | 0.7547 | 0.5866 | 0.6913 | 1.0598 |
| LIFT_YAW_PITCH | 0.0065–0.5572 | 0.0156–0.5134 | 0.7681 | 0.5866 | 0.7367 | 1.1191 |

Projected areas are 4 mm rasterized unions of boundary-enclosed bands from structurally included source configurations. They are presentation envelope estimates, not proof that every interior point is feasible.

## Increase versus C0

| Config | Δ forward X | Δ lateral | Front area Δ (%) | Right area Δ (%) |
|---|---:|---:|---:|---:|
| LIFT_ONLY | +0.0000 | +0.0000 | +0.0000 (+0.00%) | +0.0000 (+0.00%) |
| LIFT_YAW | +0.0531 | +0.0000 | +0.0439 (+6.40%) | +0.1235 (+19.05%) |
| LIFT_PITCH | +0.1462 | +0.0000 | +0.0050 (+0.72%) | +0.4116 (+63.49%) |
| LIFT_YAW_PITCH | +0.1596 | +0.0000 | +0.0504 (+7.34%) | +0.4708 (+72.63%) |

## Nested inclusion and shell

- Structural source pools contain 4320 valid boundary states; all 20 nested checks PASS.
- Zero source-pool omissions and zero per-direction inner/outer support metric regressions: C0⊆C1/C2 and C1/C2⊆C3.
- 3D shell: 6912 triangles formed only between corresponding valid angles and adjacent usable Lift slices.
- Each upper configuration shell is the union of its own and all required lower-configuration valid shells.
- No convex hull, smoothing, interpolation across invalid/no-data slices, or artificial internal fill.

## Cross-validation

| Config | Sweep max X | Fixed-orientation grid | Dense FK | Lift-slice FK | Interpretation |
|---|---:|---:|---:|---:|---|
| LIFT_ONLY | 0.6084 | 0.6771 | 0.6904 | 0.6861 | TORSO_EXPANSION_TREND_CONSISTENT |
| LIFT_YAW | 0.6615 | 0.7063 | 0.7121 | 0.7069 | TORSO_EXPANSION_TREND_CONSISTENT |
| LIFT_PITCH | 0.7547 | 0.7063 | 0.7504 | 0.7523 | TORSO_EXPANSION_TREND_CONSISTENT |
| LIFT_YAW_PITCH | 0.7681 | 0.7354 | 0.7887 | 0.7537 | TORSO_EXPANSION_TREND_CONSISTENT |

The fast one-seed directed sweep underestimates C0/C1 maximum X relative to dense positional FK, while C2/C3 are close or slightly larger. It is a presentation boundary construction, not a replacement for the validated dense/grid studies. The expansion ordering and strong Pitch/full-torso forward benefit remain consistent.

## Presentation and integrity

- Figures: 15 PNG at 1920×1080. All 2D legends are outside the plotting area and unclipped.
- Video: `presentation/boundary_sweep_workspace_demo.mp4`, 33.0 s, 1920×1080, 30 fps, H.264, 4002261 bytes; all 990 frames decoded.
- Existing manifests verified: fixed_base_workspace_manifest_sha256.txt, fixed_base_workspace_fine_manifest_sha256.txt, fixed_base_workspace_dof_ablation_manifest_sha256.txt, fixed_base_workspace_demo_manifest_sha256.txt, fixed_base_workspace_envelope_demo_manifest_sha256.txt, radial_workspace_validation_manifest_sha256.txt, workspace_projection_manifest_sha256.txt, fk_workspace_boundary_manifest_sha256.txt, lift_slice_fk_workspace_manifest_sha256.txt.
- Protected URDF/Xacro, SRDF/ACM, joint limits, kinematics and OMPL hashes unchanged.

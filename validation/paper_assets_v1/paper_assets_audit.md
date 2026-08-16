# paper_assets_v1 audit

## Scope and immutability

- Post-processing only: no ROS, MoveIt, IK, OMPL, collision-distance computation, controller, or trajectory execution was invoked.
- Raw results SHA-256: `f95048fe3e48ebffd3d1f255242a22ea03f8505dd59aa19ae3658aef8b696cf8`.
- Expected raw SHA-256: `f95048fe3e48ebffd3d1f255242a22ea03f8505dd59aa19ae3658aef8b696cf8`.
- Raw rows: 4,320; phase rows: {'PHASE1': 2896, 'PHASE2': 1304, 'PHASE3': 120}; duplicate `unique_key`: 0.
- The four replay PNGs were copied byte-for-byte to `source_png_copies/`; all panel cropping was applied only to those copies in memory.

## Figure and table provenance

1. Figure 1 uses the four copied replay PNGs. Identical pixel crop `(left=1175, top=180, right=2760, bottom=1740)` preserves equal zoom and color rendering. The panels are two independent single-axis same-target comparisons; no Yaw–Pitch synergy is claimed.
2. Figure 2 uses `mode_workspace_boundary.csv` columns `phase`, `ray_angle_deg`, `lift`, `mode`, and `last_success_distance`. Coordinates are `ΔX=d cos(θ)`, `ΔY=d sin(θ)` and the 16 ordered endpoints are closed as a polygon.
3. Figure 3 and Table 1 use `mode_workspace_area_summary.csv`: `area_m2`, `lift`, `mode`, `completed_rays`, `expected_rays`. Increase is `100 × (A_YAW_PITCH/A_LOCKED − 1)` at the same Lift.
4. Figure 4 and Table 3 use `joint_margin_summary.csv`: finite-sample `arm_joint_1_7_min_margin_[n|min|mean|max]`. IK-absence rows have no finite margin and are excluded rather than converted to zero.
5. Figure 5 uses `collision_clearance_summary.csv`, `margin_threshold_sensitivity.csv`, and `failure_taxonomy.csv`. Clearance converts metres to millimetres by `mm = 1000 × m`. Threshold areas are simulation sensitivity criteria, not hardware tolerances. `GRIPPER_ENVELOPE_INFEASIBLE` remains separate from torso/posture failure.
6. Table 2 uses four exact rows from `all_case_results.csv`, including `unique_key`, target XYZ, ray/distance/Lift/seed, selected yaw/pitch, success, margins, clearances, `failure_label`, and `classification`. Same-target inputs were asserted equal within each pair.

## Included rows and exclusions

- Boundary figure: 128 Phase-1 rows = 16 rays × 2 Lift values × 4 modes.
- Area table/figure: 8 Phase-1 aggregate rows.
- Margin and clearance: 4 and 4 Phase-1 aggregate mode rows, respectively; their source sample counts are retained in Table 3.
- Sensitivity figure: 40 stored rows (2 Lift values × 4 modes × 5 thresholds).
- Failure taxonomy figure: 8 Phase-1 classification rows; gripper-envelope counts are displayed separately.
- Same-target ablation table: 4 exact raw rows (two targets × locked/recovered mode).
- Phase 2 and Phase 3 rows are excluded from the 16-ray Phase-1 workspace figures/tables. They remain untouched in the source dataset.
- Failed cases without finite IK posture/margin/clearance are never imputed. Stored `nan`/`inf` values are not replaced with invented finite values.
- Minimum stored Phase-1 self-clearance: 0.043687139 mm; YAW_PITCH minimum: 0.043687139 mm, classified here as simulation nominal low-clearance.

## Interpretation limits

- Do not claim an exact continuous workspace; the areas are 16-ray polygon estimates.
- Do not claim hardware robustness; all results are deterministic planning-only simulation data.
- Do not claim force closure or physical grasp force.
- Do not claim Yaw–Pitch synergy from the same-target panels or YAW_PITCH-versus-LOCKED area increase. Synergy requires both single-axis modes to fail at the same target while the combined mode succeeds, which these panels do not test.
- Within the stored finite Phase-1 feasible samples, Pitch-only and Yaw+Pitch have higher mean Arm limit margin than Yaw-only; this is not a hardware-performance claim.

## Input SHA-256

- `validation/paper_main_simulation_dataset_v1/run_20260815_223216/all_case_results.csv` — `f95048fe3e48ebffd3d1f255242a22ea03f8505dd59aa19ae3658aef8b696cf8`
- `validation/paper_main_simulation_dataset_v1/run_20260815_223216/summaries/collision_clearance_summary.csv` — `3a412f2683a7c581750e581b81f3cc50280b7250027af51a0a7abdcc745a1b37`
- `validation/paper_main_simulation_dataset_v1/run_20260815_223216/summaries/failure_taxonomy.csv` — `9b7ef83997e00792c22f53a7b4fbea09a31af5535c64edfe28e3b4d619e0400b`
- `validation/paper_main_simulation_dataset_v1/run_20260815_223216/summaries/joint_margin_summary.csv` — `b3647ceed496e81f3d10621ef63d3b817516020d13f70e75c46d6a31dc342d49`
- `validation/paper_main_simulation_dataset_v1/run_20260815_223216/summaries/margin_threshold_sensitivity.csv` — `b460b903508d0b11bb3731da1a29a464925bfb7a7928e98feefd4fcb6ff9db48`
- `validation/paper_main_simulation_dataset_v1/run_20260815_223216/summaries/mode_workspace_area_summary.csv` — `bf3fe9d21315566ae71132ba84b31a9f477f24c2edf8967b0ef3583e47d42bf8`
- `validation/paper_main_simulation_dataset_v1/run_20260815_223216/summaries/mode_workspace_boundary.csv` — `fca796db01f791be92c3c81d47155702ed0365da7fa633d9b39ab603de02b4a7`
- `validation/paper_result_rviz_replay_v1/run_20260816_125837/figures/pitch_pair_locked_failure.png` — `2c21938952897a961b5e7a6d226f0b0e9b9df641792b8f6b2d4395c7fb01b33f`
- `validation/paper_result_rviz_replay_v1/run_20260816_125837/figures/pitch_pair_pitch_only_success.png` — `3e0600350321aeb6fcd5ee0278142dff7b555f9df83e78990477371abdbb56ef`
- `validation/paper_result_rviz_replay_v1/run_20260816_125837/figures/yaw_pair_locked_failure.png` — `3ad138673999422ef5c0d0847799ec8bcfff661a9644382c4515a73f470188e6`
- `validation/paper_result_rviz_replay_v1/run_20260816_125837/figures/yaw_pair_yaw_only_success.png` — `5cef5069154619891181e4a43dbb9cd54c18e00888ee7517dcb2f0d6e2703432`

## Generated-file SHA-256

The following hashes cover generated figures, tables, source copies, script, and the pre-generation input manifest. The audit file itself is hashed in `SHA256SUMS.txt` to avoid self-reference.

- `figures/figure1_same_target_single_axis_recovery_2x2.pdf` — `b6800a63e1cef828a211d0f7a07d96ea341dc9b071349e6a784651fde6181bcb`
- `figures/figure1_same_target_single_axis_recovery_2x2.png` — `c44a27117d6628d857731171e0371b3fad2ec9f85269db6b313c84c0708b3d4d`
- `figures/figure2_workspace_boundary_16ray_estimate.pdf` — `0c655a35c25f6c25e3d20c69e878a77fda977c5c7f1db7696470fca1c527dd7a`
- `figures/figure2_workspace_boundary_16ray_estimate.png` — `a4344c916296340185efc750cfae69c439292794c88ce2a8194ac1c4ca92c71c`
- `figures/figure3_workspace_area_comparison.pdf` — `ed5e4ab2276647c1ee2452ccef358d1bd7440f7ada3f000d36731667ce56bbcf`
- `figures/figure3_workspace_area_comparison.png` — `147e32136cc3197289be7694f4bcd16462e516ac700a5ea4f0c8653fd43c6d3f`
- `figures/figure4_arm_joint_margin_comparison.pdf` — `5aaf7d6b59311a94f6f67d4ac75af5b9aa7b19a054b01af1e32bcd7400bd8eb7`
- `figures/figure4_arm_joint_margin_comparison.png` — `6abbea96f25e14a8d1dcc74fa7bfcf7d4d99a3213af2afe140bd7be4a2b3b7a9`
- `figures/figure5_robustness_and_failure_taxonomy.pdf` — `0d449aa63dd1b47ef7b388de8b0615c0fba7ec0508233522d219bdac7daf3b8a`
- `figures/figure5_robustness_and_failure_taxonomy.png` — `ac5208954374c0ba60a2e3f355245c40b7b97d946ee8837222311f5c02321a86`
- `generate_paper_assets_v1.py` — `64e42556830ae30472c1f38e3a0bae9877e98180d0dd9dfc741d81d81bae282d`
- `input_manifest_before.sha256` — `e8b89057935548e5b28d447896ebe4e26152108387cd091045a85cabfc204df1`
- `source_png_copies/pitch_pair_locked_failure.png` — `2c21938952897a961b5e7a6d226f0b0e9b9df641792b8f6b2d4395c7fb01b33f`
- `source_png_copies/pitch_pair_pitch_only_success.png` — `3e0600350321aeb6fcd5ee0278142dff7b555f9df83e78990477371abdbb56ef`
- `source_png_copies/yaw_pair_locked_failure.png` — `3ad138673999422ef5c0d0847799ec8bcfff661a9644382c4515a73f470188e6`
- `source_png_copies/yaw_pair_yaw_only_success.png` — `5cef5069154619891181e4a43dbb9cd54c18e00888ee7517dcb2f0d6e2703432`
- `tables/table1_lift_16ray_estimated_area.csv` — `c334bfc257bc6f75fd8a0187597f59d9ecca93c609fa3a4b18a86d8926f5648d`
- `tables/table1_lift_16ray_estimated_area.md` — `31ea90d8df7ce37af22f9cc1b2d27504c3474c5e3d962bef16a3253ccb14a224`
- `tables/table2_same_target_axis_ablation.csv` — `90bd7d376af7dd70a231a40e134a92f46527afb0fb6d7520ffdd4060c6e46b59`
- `tables/table2_same_target_axis_ablation.md` — `5f4a4ea504a9f8885531abd43a755acaf5dbd8b2e763b2b72970ef510212e2c6`
- `tables/table3_joint_margin_collision_clearance.csv` — `c2b56393cbabceedf34449dbbc5d20d75f88fce1ae550621e89178067db9ea61`
- `tables/table3_joint_margin_collision_clearance.md` — `ff34e53fe1908ed8ce0c13136139abe35afa9e7a7a01f5963d38c3a2323d03c1`

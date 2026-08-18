# Radial workspace validation audit

## Scope and immutable sources

- Purpose: lightweight presentation-oriented boundary validation; not a full workspace recomputation.
- Configurations: C0 `ARM + LIFT` and C3 `ARM + LIFT + WAIST_YAW + WAIST_PITCH` only.
- Base/TCP frames: `base_link` / `openarm_left_hand_tcp`; +X forward, +Y left, +Z up.
- Source comparison SHA-256: `5d7f1ea256b99ee0a36b1e8eab9250b34d89bcf9dce3c9fc61671eccd10f96b2`.
- Coarse manifest SHA-256: `4c6a188e19ad273159600f27d8f67c6603da1de5794910240e6d35f6e715c90e`.
- Fine manifest SHA-256: `5fea859d75d0fb4b7f7b2fc7c866fae56358ecb3c674ae642ca7220b9f7164c4`.
- DOF ablation manifest SHA-256: `982e1b6a2615a7f107356dc046dcea5143cf01cadfd7c807dd21528214caa4d2`.
- Envelope demo manifest SHA-256: `93ce01da4215295ce2d365961eb655bff5ed9ba0b4cffea46ff70e8933646bc7`.
- Reference origin in base_link: (0.214583333333333, 0.21, 1.175) m.
- Radial limits/step: 0.15–0.8 m / 0.02 m.
- Physical points / configuration evaluations: 92 / 184.
- Maximum IK seeds: 50.
- Internal anomaly revalidation: 2 points, up to 100 seeds each.

## Ray vectors

- FRONT: `1;0;0`
- FRONT_LEFT: `0.911921505175106;0.410364677328798;0`
- FRONT_RIGHT: `0.911921505175106;-0.410364677328798;0`
- FRONT_UP: `0.838443616300637;0;0.544988350595414`
- FRONT_DOWN: `0.838443616300637;0;-0.544988350595414`

## Feasible intervals and holes

| Configuration | Ray | First (m) | Last (m) | Intervals | Holes | Largest hole (m) | PASS/FAIL |
|---|---|---:|---:|---:|---:|---:|---:|
| LIFT_ONLY | FRONT | 0.19 | 0.47 | 1 | 0 | 0 | 15/3 |
| LIFT_ONLY | FRONT_LEFT | 0.21 | 0.43 | 1 | 0 | 0 | 12/4 |
| LIFT_ONLY | FRONT_RIGHT | 0.21 | 0.47 | 1 | 0 | 0 | 14/2 |
| LIFT_ONLY | FRONT_UP | 0.23 | 0.55 | 1 | 0 | 0 | 17/4 |
| LIFT_ONLY | FRONT_DOWN | 0.23 | 0.53 | 2 | 1 | 0.02 | 15/6 |
| LIFT_YAW_PITCH | FRONT | 0.19 | 0.47 | 1 | 0 | 0 | 15/3 |
| LIFT_YAW_PITCH | FRONT_LEFT | 0.21 | 0.45 | 1 | 0 | 0 | 13/3 |
| LIFT_YAW_PITCH | FRONT_RIGHT | 0.21 | 0.49 | 1 | 0 | 0 | 15/1 |
| LIFT_YAW_PITCH | FRONT_UP | 0.23 | 0.57 | 1 | 0 | 0 | 18/3 |
| LIFT_YAW_PITCH | FRONT_DOWN | 0.23 | 0.55 | 1 | 0 | 0 | 17/4 |

Each interval is the exact contiguous 20 mm PASS sample run. Holes include only FAIL runs bounded by PASS on both sides; exterior FAIL samples are not mislabeled as holes.

### Explicit intervals

- LIFT_ONLY / FRONT / #1: 0.19–0.47 m (15 samples).
- LIFT_ONLY / FRONT_LEFT / #1: 0.21–0.43 m (12 samples).
- LIFT_ONLY / FRONT_RIGHT / #1: 0.21–0.47 m (14 samples).
- LIFT_ONLY / FRONT_UP / #1: 0.23–0.55 m (17 samples).
- LIFT_ONLY / FRONT_DOWN / #1: 0.23–0.49 m (14 samples).
- LIFT_ONLY / FRONT_DOWN / #2: 0.53–0.53 m (1 samples).
- LIFT_YAW_PITCH / FRONT / #1: 0.19–0.47 m (15 samples).
- LIFT_YAW_PITCH / FRONT_LEFT / #1: 0.21–0.45 m (13 samples).
- LIFT_YAW_PITCH / FRONT_RIGHT / #1: 0.21–0.49 m (15 samples).
- LIFT_YAW_PITCH / FRONT_UP / #1: 0.23–0.57 m (18 samples).
- LIFT_YAW_PITCH / FRONT_DOWN / #1: 0.23–0.55 m (17 samples).

### Internal holes

- LIFT_ONLY / FRONT_DOWN / #1: 0.51–0.51 m; sampled width 0.02 m.

## Representative RobotState and presentation evidence

- First/last feasible RobotState rows: 20; each state passed IK, joint bounds, exact-bound, fixed-orientation, and self-collision checks.
- Demo uses static switching between validated states; it does not claim a planned or executable trajectory.
- Full video: `/home/openarm/humanoid_sim_ws/presentation/radial_workspace_validation_demo.mp4`, 44.943 s, 1920x1080, H.264, 4789813 bytes.
- Short video: `/home/openarm/humanoid_sim_ws/presentation/radial_workspace_validation_demo_short.mp4`, 34.930 s, 1920x1080, H.264, 3759247 bytes.
- Screenshots: `presentation/radial_front_c0.png`, `radial_front_c3.png`, `radial_hole_example.png`, `radial_min_max_pose.png`.

## Safety

- AMR/base fixed: yes.
- Trajectory execution: **false**.
- Controller: **false**.
- ros2_control: **false**.
- Hardware: **false**.
- Box/environment collision objects: none.

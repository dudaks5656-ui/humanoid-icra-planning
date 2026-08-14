# ICRA project handoff — humanoid confined-space extraction

작성 시각: 2026-08-14 (Asia/Seoul)  
작업공간: `/home/openarm/humanoid_sim_ws`  
작성 원칙: 이 문서는 현재 파일 시스템의 YAML, CSV, Markdown, Xacro, URDF 및 C++/launch 소스를 읽어 대조한 인계 문서다. 기억만으로 수치를 보충하지 않았다.

## 상태 표기

- **VERIFIED**: 현재 소스 또는 결과 파일에서 직접 확인했다.
- **ACTIVE**: 현재 실행 경로가 실제로 읽는 값이다.
- **PROVISIONAL**: 구현·방법 개발에는 사용 중이지만 논문 최종값, 물리 측정값 또는 안전값으로 확정되지 않았다.
- **UNVERIFIED**: 현재 작업공간만으로는 사용자 최종 승인 또는 물리적 타당성을 입증할 수 없다.
- **HISTORICAL**: 보존해야 하지만 현재 최종 장면/방법의 결과가 아니다.

## 1. 프로젝트 결론과 연구 질문

### 최종 연구 주제

**PROVISIONAL — 명시적인 ICRA 제목/연구질문 원본은 작업공간에 없다.** 현재 코드와 `pilot_conclusion.md`가 지시하는 연구 방향은 다음과 같다.

> 상부 개방 협소 박스에서 OpenArm 기반 휴머노이드가 50 mm 물체를 인출할 때, 단계 제약 기준 경로(stage-constrained reference)를 따르다가 실패한 국소 구간에서만 Lift/Yaw/Pitch를 선택적으로 사용해 복구하는 stage-aware local recovery 계획법.

기존의 “전역 OMPL에서 Lift-only와 Lift–Yaw–Pitch를 직접 비교”하는 방식은 반복 실험에서 Yaw/Pitch의 유의미한 우위를 보이지 못했다. 따라서 논문의 핵심은 전역 자유도 추가 자체가 아니라, **어느 단계의 어떤 실패에 어떤 torso 자유도를 얼마나 작게 투입해야 하는가**로 전환되어야 한다.

### 핵심 연구 질문

1. 단계 의미를 보존하는 reference trajectory가 협소공간 인출의 재현성과 실패 진단성을 높이는가?
2. reference의 실패 stage/waypoint 주변에서만 Lift/Yaw/Pitch를 허용하면 전역 whole-body OMPL보다 성공률·계획시간·clearance·torso motion cost를 개선하는가?
3. 실패 유형과 stage 정보를 사용해 Lift, Yaw, Pitch의 부분집합을 선택할 수 있는가?
4. 같은 고정 grasp와 object lifecycle을 사용하면서, 성공이 단순한 OMPL 확률 변동이 아니라 구조적인 local recovery임을 어떻게 입증할 것인가?

위 질문과 문구는 **PROVISIONAL**이다. 논문 제목, 주장 범위, 최종 hypothesis는 다음 Codex가 실험 설계 전에 사용자에게 확인해야 한다.

## 2. 로봇 플랫폼

### 활성 플랫폼 구성

- 고정 world 프레임의 AMR base. AMR 이동은 계획 자유도에 포함되지 않는다.
- 수직 1-DOF `lift_joint`.
- 허리 `waist_yaw_joint`, `waist_pitch_joint` 2-DOF.
- 좌우 OpenArm v1.0/v10 7-DOF arm 두 개.
- 좌우 parallel-link gripper. 각 gripper는 독립 prismatic joint 하나와 mimic/passive joint 하나를 갖는다.
- Head와 상체 외장 케이싱은 현재 모델 범위에서 제외되어 있다.
- 활성 robot name은 `humanoid_sim_provisional_v10`; OpenArm 물리 버전 최종 식별은 **UNVERIFIED**다.
- 현재 시스템은 **planning-only**다. `ros2_control`, controller, hardware interface, AMR/OpenArm driver, Serial/CAN/USB, trajectory execution은 사용하지 않는다.

활성 kinematic chain:

```text
world
└─[fixed] base_link / AMR
  └─[fixed] lift_fixed_link
    └─[prismatic] lift_moving_link
      └─[revolute] waist_yaw_link
        └─[revolute] waist_pitch_link
          ├─[two fixed joints] left OpenArm 7-DOF + parallel gripper
          └─[two fixed joints] right OpenArm 7-DOF + parallel gripper
```

근거: `src/humanoid_sim_description/urdf/*.xacro`, `src/humanoid_sim_description/validation/README_KO.md`, `SOURCE_PROVENANCE.md`.

## 3. 확정 관절 범위

### Lift/Yaw/Pitch

현재 활성 Xacro가 읽는 값과 expanded URDF가 일치한다.

| Joint | Type / axis | Lower | Upper | 방향/비고 |
|---|---|---:|---:|---|
| `lift_joint` | prismatic, `[0,0,-1]` | 0.0 m | 0.700 m | q=0이 최상단, +q는 world -Z, 즉 아래쪽 |
| `waist_yaw_joint` | revolute, `[0,0,1]` | -0.174533 rad | +0.174533 rad | 약 -10°..+10°; 위에서 볼 때 -는 CW, +는 CCW |
| `waist_pitch_joint` | revolute, `[0,1,0]` | -0.174533 rad | +0.785398 rad | 약 -10° 후방..+45° 전방 |

Position range와 방향은 사용자 RViz 확인 기록이 있다. 단, effort/velocity는 `provisional_dynamics.yaml`의 미측정 placeholder이므로 논문·제어·안전값으로 사용하면 안 된다.

### 활성 OpenArm 관절 범위

아래는 raw 공통 YAML이 아니라 좌우 반사/offset이 적용된 **활성 expanded URDF** 기준이다. 단위는 rad이다.

| Joint | Left lower..upper | Right lower..upper |
|---|---:|---:|
| joint1 | -3.490659 .. +1.396263 | -1.396263 .. +3.490659 |
| joint2 | -3.3161253267948965 .. +0.17453267320510335 | -0.17453267320510335 .. +3.3161253267948965 |
| joint3 | -1.570796 .. +1.570796 | -1.570796 .. +1.570796 |
| joint4 | 0.0 .. +2.443461 | 0.0 .. +2.443461 |
| joint5 | -1.570796 .. +1.570796 | -1.570796 .. +1.570796 |
| joint6 | -0.785398 .. +0.785398 | -0.785398 .. +0.785398 |
| joint7 | -1.570796 .. +1.570796 | -1.570796 .. +1.570796 |

각 독립 `finger_joint1`과 mimic `finger_joint2`의 URDF 범위는 0.0..0.044 m다. `finger_joint2`는 독립 계획변수가 아니다. q=0 부근에서 opposing-finger mesh collision이 검출되며, q=0.0088..0.044 m 샘플에서는 해소됐다. 이를 이유로 global ACM을 넓혀서는 안 된다.

근거: `cad_frame_transforms.yaml`, `openarm_v1.0/config/arm/joint_limits.yaml`, gripper `joint_limits.yaml`, `validation/validated_mount_transition_after.urdf`, `validation/joint_limit_violation_audit.md`.

## 4. 좌표계와 축 방향

- ROS/world convention: **+X forward, +Y left, +Z up**, metre/radian.
- CAD-to-ROS mapping: ROS +X = CAD +Y, ROS +Y = CAD -X, ROS +Z = CAD +Z.
- `world_to_base_joint`: identity fixed transform.
- Lift top-zero origin in `base_link`: `[0.2549756914527, 0.0002288135233, 1.198]` m.
- Lift top-zero to Yaw: identity.
- Yaw to Pitch: `[-0.0435192366720, 0.0349688054576, 0.1538233064808]` m.
- 사용자 검증 shoulder mount, `waist_pitch_link` frame:
  - left `[0.0435192366720, -0.0049688054576, 0.2020497164153]` m
  - right `[0.0435192366720, -0.0649688054576, 0.2020497164153]` m
- OpenArm link0 adapter RPY: left `[-1.5708,0,0]`, right `[+1.5708,0,0]`.
- TCP `openarm_left_hand_tcp`는 `openarm_left_link7`의 identity fixed child다.
- TCP local +Z가 approach axis, finger closing axis는 local ±Y다.
- 최종 top-entry grasp에서는 TCP local +Z → world -Z, local +Y → world +Y, local +X → world -X다.

Mesh frame은 `amr_base.stl`/`lift_fixed_structure.stl`=`CS_AMR_BASE`, `lift_moving_structure.stl`=`CS_LIFT_TOP_ZERO`, `waist_yaw.stl`=`CS_YAW_AXIS`, `waist_pitch.stl`=`CS_PITCH_AXIS`이며 모두 mm source에 URDF scale 0.001을 사용한다.

## 5. ROS 2 Humble, Xacro, URDF, MoveIt 현황

- OS/ROS validation 기록: Ubuntu 22.04, ROS 2 Humble. `/opt/ros/humble` prefix가 현재 존재한다.
- Colcon packages: `humanoid_description`, `humanoid_extraction_experiments`, `humanoid_sim_description`, `humanoid_sim_moveit_config`, `openarm_description`.
- **활성 description**: `humanoid_sim_description/urdf/humanoid_sim.urdf.xacro`.
- `humanoid_description`은 이전 primitive/provisional 모델과 검증 기록이다. 최종 실험의 활성 모델로 혼동하지 말 것.
- URDF root는 `world`; fixed `world_to_base_joint`; SRDF virtual joint 없음.
- MoveIt independent variables 19개: torso 3 + left arm 7 + right arm 7 + 독립 finger 2.
- SRDF groups: `torso`, `left_arm`, `right_arm`, 두 gripper, `left_arm_with_torso`, `right_arm_with_torso`, `dual_arm`, `dual_arm_with_torso`, `whole_body`.
- Kinematics: KDL plugin. Arm timeout 0.05 s, arm-with-torso timeout 0.10 s, resolution 0.005.
- Planner: OMPL `geometric::RRTConnect`; baseline `longest_valid_segment_fraction=0.01`; fine 비교 profile 0.0025는 PROVISIONAL.
- 실행 금지 설정: `allow_trajectory_execution=false`, `moveit_manage_controllers=false`, `MoveGroupExecuteTrajectoryAction` disabled.
- Collision detector: MoveIt/FCL. Global SRDF ACM은 23쌍이며, 원래 adjacent 22쌍에 사용자 검증 고정 mount contact `waist_pitch_link`↔`openarm_left_link0` 한 쌍만 추가됐다.
- latest build log(2026-08-14 12:24)는 `openarm_description`, `humanoid_sim_description`만 선택 빌드했다. 실험 executable symlink는 2026-08-12 빌드 산출물을 가리킨다. 인계 후 전체 clean-room build 검증은 아직 하지 않았다.

## 6. 상부 개방 박스와 50 mm 물체

### 현재 활성 geometry

| 항목 | 값 | 상태 |
|---|---|---|
| 내부 폭 Y | 0.600 m | user-defined, VERIFIED |
| 내부 깊이 X | 0.400 m | user-defined, VERIFIED |
| 내부 높이 Z | 0.150 m | user-defined, VERIFIED |
| 내부 X 범위 | [0.475, 0.875] m | derived, VERIFIED |
| 내부 Y 범위 | [-0.100, 0.500] m | derived, VERIFIED |
| 내부 Z 범위 | [0.940, 1.090] m | derived, VERIFIED |
| 내부 중심 | [0.675, 0.200, 1.015] m | ACTIVE |
| wall/floor thickness | 0.025 m | PROVISIONAL_WALL_THICKNESS |
| top | open, ceiling 없음 | VERIFIED |

다섯 collision object는 floor/front/back/left/right wall이다. 외곽 AABB는 X `[0.450,0.900]`, Y `[-0.125,0.525]`, Z `[0.915,1.090]` m다. 초기 robot state와 box의 FCL collision은 없었다. 절대 배치가 현재 소스와 모든 top-open 실험에서 고정되어 사용된 것은 VERIFIED이지만, 최초 geometry audit에 “user visual approval pending”이 남아 있으므로 **논문 최종 물리 배치 승인 여부는 UNVERIFIED**다.

### Pick object

- 50 × 50 × 50 mm cube.
- center `[0.675, 0.200, 0.965]` m.
- AABB X `[0.650,0.700]`, Y `[0.175,0.225]`, Z `[0.940,0.990]` m.
- floor top Z=0.940 m와 정확히 접하고 vertical wall과는 겹치지 않는다.
- front/back clearance 각 0.175 m, right/left clearance 각 0.275 m.

## 7. 확정 grasp 설정

현재 `validation/reference_grasp_50mm.yaml`과 `top_open_reference_scene.yaml`이 사용하는 고정 kinematic reference grasp:

- 이름: `KINEMATIC_REFERENCE_GRASP_50MM`.
- 적용 범위: central object에서 선택하고 모든 target/ablation에 동일 grasp를 재사용.
- orientation: RPY `[0, pi, 0]`, quaternion xyzw `[0,1,0,0]`.
- approach: world -Z, closing: object/world ±Y.
- `q_open = 0.044 m`.
- `q_contact_50mm = 0.0327611885070801 m`.
- grasp center height: object bottom 위 **0.050 m**.
- grasp TCP pose: `[0.675, 0.200, 1.12545987646484]` m, quaternion `[0,1,0,0]`.
- TCP-to-grasp-center: `0.13545987646484376 m`.
- finger-floor clearance: `0.002539612 m`.
- 실제 inward-face vertical overlap: `0.0124168090820314 m`.
- symmetric opposing contact, measured penetration 0 m.

중요한 제한:

- q_open은 simulation planning aperture이며 hardware fully-open 검증값이 아니다.
- q_contact는 `KINEMATIC_CONTACT_CONFIGURATION_FOR_50MM_OBJECT`일 뿐 force closure, friction, contact pressure, grasp force, hardware stability를 검증하지 않았다.
- 이전 15 mm development overlap criterion은 12.4168 mm mesh patch 때문에 실패했다. 이후 50 mm 높이는 **kinematic path planning only**로 명시적으로 채택됐다. 이 이력은 삭제하거나 “물리 파지 성공”으로 바꾸면 안 된다.

## 8. 주요 예비실험과 결과

| 실험 | 결과 | 핵심 파일 |
|---|---|---|
| OpenArm description 복사/무결성 | 153 files 복사, 92 Xacro/mesh hash mismatch 0. 단 physical model 식별은 provisional v10 | `SOURCE_PROVENANCE.md` |
| CAD mesh/Xacro static validation | 5개 humanoid STL의 unit/frame 검증, Xacro/XML/check_urdf 통과 | `src/humanoid_sim_description/validation/README_KO.md`, `validation_report.csv/json` |
| Lift/Yaw/Pitch measured-limit RViz validation | 범위와 표시 방향 확인 성공; effort/velocity 및 일부 geometry는 provisional | `src/humanoid_description/validation/measured_joint_limits_validation.md` |
| MoveIt/SCM baseline | planning-only 구성 성공; 10,000 random states, 196 protected pairs, 잘못 disable된 protected pair 0. 초기 3 collision 발견 | `validation/moveit_implementation_validation.md`, `validation/self_collision_matrix_audit.txt` |
| Arm mount/ACM transition | user-validated symmetric mount 유지; fixed left mount contact 1쌍만 ACM 추가; post-check PASS | `validation/validated_mount_transition.md`, `validation/post_acm_collision_check.md`, `validation/acm_scope_verification.csv` |
| Front-open grasp orientation | side-entry RPY `[0,pi/2,0]`가 geometry 기준 선택됐으나 당시 pre-grasp envelope에서 두 finger-target collision로 3회 모두 실패 | `validation/grasp_orientation_audit.md`, `validation/orientation_candidates/` |
| Front-open grasp/object lifecycle | q_open geometry, task-scoped finger contact, attach lifecycle 구현 성공. 세 planning run은 full five-stage 성공 0; furthest candidates는 EXTRACTION IK failure | `validation/grasp_geometry_audit.md`, `validation/object_lifecycle_verification.md` |
| Front-open extraction endpoint correction | object-first equation으로 5/10/20/30 mm outside clearance 모두 IK 존재; 최소 5 mm 선택. planning repeat은 stochastic | `validation/extraction_endpoint_audit.md`, `validation/extraction_clearance_search.csv` |
| Boundary search | 35 targets 중 geometry-feasible 30; LIFT_ONLY 27/30, LIFT_YAW_PITCH 30/30. 그러나 세 차이는 structural recovery가 아니라 OMPL 실패 | `validation/single_case_extraction_summary.md`, `target_boundary_search.csv`, `lift_only_vs_lift_yaw_pitch.csv`, `ik_multistart_audit.csv` |
| Differential repeat | 3 targets × 2 modes × 3 budgets × 20 = 360 trials 완료. 모두 `NO_MEANINGFUL_TORSO_ADVANTAGE` | `validation/pilot_experiment_20260812/differential_target_analysis.md`, `differential_target_repeat_trials.csv`, `planning_budget_sensitivity.csv` |
| Front-open offline reference | 10회 중 5 dense-valid, trial 7 선택(Lift 0.1, Yaw/Pitch 0). 단 active joint margin 0, orientation tolerance 경고가 있어 HISTORICAL method-development reference | `validation/reference_path_audit.md`, `reference_trajectory.csv`, `reference_stage_waypoints.yaml` |
| Top-open box geometry/visibility | 5-face geometry와 50 mm cube 확정, 초기 robot-box collision 없음. RViz invisibility 원인은 topic/config 문제 | `validation/top_open_box_geometry_audit.md`, `top_open_box_visibility_audit.md`, `top_open_50mm_object_geometry_audit.md` |
| Top-open center reachability, 최초 center grasp | APPROACH/PRE_GRASP 가능, GRASP 4,114 IK 모두 finger-floor collision; gate 실패 | `validation/top_open_center_reachability_audit.md`, `top_open_center_reachability.csv` |
| Corrected grasp reachability | 47.460388 mm geometry에서 세 stage 모두 LIFT_ONLY 및 LYP feasible; best LIFT_ONLY=0.35 m. 이 값은 이후 final kinematic grasp 50 mm로 대체됨 | `validation/top_open_center_reachability_grasp_corrected.md/.csv` |
| 50 mm height audits | 47.5..50.0 mm 모두 IK/free였으나 actual overlap 12.4168 mm로 당시 15 mm criterion 실패. 이후 50 mm를 kinematic-only reference로 선택 | `validation/reference_grasp_fine_height_audit.md`, `reference_grasp_50mm.yaml` |
| Top-open random OMPL reference | 20/20 trials, full+dense valid 2개(trials 4,9), required 5개 미달로 reference 미선정 | `validation/top_open_reference_path_audit.md`, `top_open_reference_generation_trials.csv`, `top_open_reference_trajectory.csv` |
| Stage-constrained prototype | 10/10 실패. Lift 0.30에서 vertical descent 85.2941%까지 진행; 최종 trajectory 미선정 | `validation/stage_constrained_reference_audit.md`, `stage_constrained_reference_trials.csv`, `stage_constrained_reference_trajectory.csv` |
| Grasp-seeded reverse continuation | 10/10 실패. Lift 0.35, 1 mm spacing에서 reverse segment 91.6667%, top endpoint 약 14 mm 전 IK failure | `validation/grasp_seeded_reference_audit.md`, `grasp_seeded_reference_trials.csv`, `grasp_seeded_reference_trajectory.csv` |

모든 planning 실험은 trajectory execution 없이 수행됐다.

## 9. 기존 전역 OMPL에서 Yaw/Pitch 우위가 없었던 결과

Boundary search의 표면적 결과는 30 feasible targets에서 LIFT_ONLY 27/30, LIFT_YAW_PITCH 30/30이었다. 그러나 차이가 난 세 target 모두 LIFT_ONLY에도 collision-free IK가 있었고 failure가 OMPL stochastic planning failure였으므로 structural torso recovery로 인정되지 않았다.

360-repeat 결과:

| Target | Proposed pose Lift/Yaw/Pitch | LIFT_ONLY success 2/5/10 s | LYP success 2/5/10 s | 분류 |
|---|---|---|---|---|
| `[0.500,0.300,1.250]` | `0 / 0 / 0` | 0.55 / 0.30 / 0.60 | 0.40 / 0.40 / 0.45 | NO_MEANINGFUL_TORSO_ADVANTAGE |
| `[0.525,0.305,1.250]` | `0 / 0 / 0` | 0.55 / 0.50 / 0.45 | 0.45 / 0.25 / 0.50 | NO_MEANINGFUL_TORSO_ADVANTAGE |
| `[0.575,0.315,1.250]` | `0 / -0.0872665 / 0` | 0.60 / 0.60 / 0.60 | 0.35 / 0.30 / 0.55 | NO_MEANINGFUL_TORSO_ADVANTAGE |

첫 두 target은 양 mode의 torso pose가 실질적으로 동일했다. nonzero Yaw -5°를 쓴 세 번째도 10 s에서 0.55 대 0.60으로 우위가 없었다. 22개의 returned stage trajectory는 exact bounds recheck에서 `TRAJECTORY_JOINT_LIMIT`로 거부됐지만 raw point array가 저장되지 않아 joint/excess를 사후 판정할 수 없다. 이 pilot을 논문 최종 성능 결과로 사용하면 안 된다.

## 10. 상부 개방 박스 기준 경로 생성 결과

### Random/global top-open generator

- Stages: `TOP_APPROACH`, `VERTICAL_DESCENT`, `GRASP`, `LIFT_CLEAR`, `TRANSFER_OUTSIDE`.
- 20 full-task trials 중 dense-valid 2개(trials 4, 9).
- trial 4: initial Lift 0.35, environment clearance 0.000109414 m, self clearance 0.00065926 m.
- trial 9: initial Lift 0.25, environment clearance 0.000581084 m, self clearance 0.00065926 m.
- required comparison set 5개를 얻지 못했고 둘 다 active-joint margin 0이라 **reference 미선정**.
- `top_open_reference_trajectory.csv`는 trajectory가 아니라 `NOT_SELECTED,FAILED_TO_OBTAIN_FIVE_DENSE_VALID_CANDIDATES_2_OF_5` 상태 파일이다.

### Stage-constrained prototype

- OMPL TOP_APPROACH → Cartesian vertical descent → 10-step finger closure → Cartesian lift → OMPL transfer 구조가 이미 구현돼 있다.
- 10회 모두 실패. Lift 0.35/0.40은 qualifying TOP_APPROACH branch 0; Lift 0.20/0.25/0.30은 continuous descent IK가 각각 약 26.47%/55.88%/85.29%에서 끊겼다.
- `stage_constrained_reference_trajectory.csv`는 `NOT_SELECTED,NO_APPROVABLE_STAGE_CONSTRAINED_REFERENCE`다.

### Grasp-seeded prototype

- GRASP에서 TOP_APPROACH로 reverse continuation한 뒤 뒤집어 descent를 만드는 방법.
- Lift 0.35에서 24 collision-free grasp branches를 찾고 1 mm step으로 91.6667%까지 진행했지만 top endpoint 약 14 mm 전 IK가 끊겼다.
- `grasp_seeded_reference_trajectory.csv`는 `NOT_SELECTED,NO_APPROVABLE_GRASP_SEEDED_REFERENCE`다.

따라서 현재 top-open 장면에는 승인된 reference trajectory가 **없다**. `reference_trajectory.csv`의 성공 경로는 front-open/HISTORICAL 장면이므로 혼용하면 안 된다.

## 11. 보호해야 할 파일

다음은 변경 전 backup/hash와 결과 비교 없이 수정하면 안 된다.

### 활성 robot description과 mount

- `src/humanoid_sim_description/urdf/humanoid_sim.urdf.xacro`
- `src/humanoid_sim_description/urdf/amr_base.xacro`
- `src/humanoid_sim_description/urdf/lift.xacro`
- `src/humanoid_sim_description/urdf/waist.xacro`
- `src/humanoid_sim_description/urdf/openarm_arms_adapter.xacro`
- `src/humanoid_sim_description/config/cad_frame_transforms.yaml`
- `src/humanoid_sim_description/config/validated_arm_mount_alignment.yaml`
- `src/humanoid_sim_description/meshes/*.stl`
- `src/openarm_description/assets/robot/openarm_v1.0/**`
- `src/openarm_description/assets/end_effector/parallel_link/**`

### MoveIt/SRDF/ACM/limits

- `src/humanoid_sim_moveit_config/config/humanoid_sim.srdf`
- `src/humanoid_sim_moveit_config/config/joint_limits.yaml`
- `src/humanoid_sim_moveit_config/config/kinematics.yaml`
- `src/humanoid_sim_moveit_config/config/ompl_planning.yaml`
- `src/humanoid_sim_moveit_config/config/ompl_planning_fine.yaml`
- `src/openarm_description/assets/robot/openarm_v1.0/config/arm/joint_limits.yaml`
- `src/openarm_description/assets/end_effector/parallel_link/config/joint_limits.yaml`

### 고정 scene/grasp와 증거

- `src/humanoid_extraction_experiments/config/top_open_box_600x400x150.yaml`
- `src/humanoid_extraction_experiments/config/top_open_reference_scene.yaml`
- `validation/reference_grasp_50mm.yaml`
- `validation/*manifest_sha256.txt`
- `validation/pilot_experiment_20260812/**`
- 모든 `*_audit.md`, 결과 CSV와 `validation/backups/**`; 덮어쓰지 말고 새 run-id 경로를 사용한다.

핵심 현재 SHA-256:

```text
e0bcfb64859d94203eeec45f823dec8d238d72bec6809d81c2c99091942be4e4  humanoid_sim.urdf.xacro
b25f74e2b1facea2d106f7d58a43cc086eecc8744f5d71e92d02d6f9d683a280  openarm_arms_adapter.xacro
d42520f6ec0c6ee92d60f1238ee6373b2bddc26277f112fa036f25e6a5044840  cad_frame_transforms.yaml
6b3fb9b254a7d8438ab77eb747260a20c49d3ea6078a46de1e3ea1d444d527aa  validated_arm_mount_alignment.yaml
7892b122fe28e91650f8919a66c256ebc55d3b34472e006701a63cadab5484e8  humanoid_sim.srdf
1e851fa791dab1c95eeba4a698e620c0334afd12a0c9964b774f24868d03ccfb  MoveIt joint_limits.yaml
067d96cd7a868ce7359629d299b6c211e4038e00b1cccf1d5d1eb496f9e5be8d  top_open_box_600x400x150.yaml
730b70b98aaba021f69f4f7770279bf629f9dd8bc7e138829a6bf93e6af9ce5f  top_open_reference_scene.yaml
974fa50c7446bc6cd430d0adfbc37a4ecfddd57adce68e64e26a88dfa102245e  reference_grasp_50mm.yaml
```

## 12. 남아 있는 기술 문제

1. 승인된 top-open reference trajectory가 없다.
2. Cartesian vertical descent의 continuous IK branch가 끊긴다. best reverse result도 top endpoint 약 14 mm 전에 실패했다.
3. random-valid 두 경로는 active-joint margin 0이고 clearance가 0.109/0.581 mm 수준이라 robust reference로 부적합하다.
4. q_open은 upper bound라 all-variable margin이 항상 0이다. 물리 open convention도 UNVERIFIED다.
5. q_contact/grasp height는 kinematic-only이며 force closure와 hardware stability가 UNVERIFIED다.
6. OpenArm physical model revision은 provenance 문서상 provisional v10/v1.0이다.
7. dynamics effort/velocity, box wall/floor thickness, 일부 최종 실험 설정은 PROVISIONAL이다.
8. top-open absolute box placement의 논문 최종 사용자 승인 기록이 명확하지 않다.
9. Humble `move_group`가 결과 저장 후 shutdown에서 class-loader warning 뒤 SIGSEGV(-11)를 반복한다. 실험 process는 먼저 정상 종료했지만 정리 문제는 남아 있다.
10. 과거 22 joint-limit rejection의 raw trajectory points가 없어 원인 joint를 복구할 수 없다. 이후 모든 rejected raw trajectory를 저장해야 한다.
11. stage-aware local recovery 구현은 아직 없다. 소스 검색에서 독립 implementation을 찾지 못했다.
12. source 25개, validation 66개 파일에 `/home/openarm`, Windows OneDrive path 또는 hostname 흔적이 있다. cloud/GitHub 이식성이 낮다.
13. 정밀 visual mesh와 collision mesh 분리가 불완전해 계산 비용이 커질 수 있다.
14. ICRA 최종 target grid, trial 수, seed policy, 통계 검정, hardware 범위는 UNVERIFIED다.

## 13. 앞으로 구현할 stage-constrained reference planner

현재 `stage_constrained_reference_generator.cpp`와 `grasp_seeded_reference_generator.cpp`는 실패 기록을 가진 **v1 prototype**으로 보호한다. 새 구현은 별도 v2 source/executable/output prefix로 만든다.

권장 v2 절차(**PROVISIONAL DESIGN**):

1. `reference_grasp_50mm.yaml`, top-open scene, global ACM, exact joint limits를 불변 입력으로 읽고 hash를 run manifest에 기록한다.
2. stage semantics를 고정한다: OMPL `TOP_APPROACH`; fixed orientation/XY Cartesian `VERTICAL_DESCENT`; 10-step finger-only `GRASP`; attached-object Cartesian `LIFT_CLEAR`; OMPL `TRANSFER_OUTSIDE`.
3. GRASP pose에서 collision-free IK branches를 다중 seed로 구하고 active-joint margin, self/environment clearance로 rank한다.
4. GRASP→TOP reverse continuation과 TOP→GRASP forward continuation을 동시에 생성한다. 기존 best branch의 14 mm gap 주변에서 seed interpolation, neighboring IK branch graph 또는 constrained local connection을 시도한다. TCP orientation과 vertical line constraint를 완화하지 않는다.
5. torso reference는 먼저 Yaw=Pitch=0으로 유지한다. Lift 후보는 현 prototype의 0.20/0.25/0.30/0.35/0.40 m를 재현한 뒤, 변경 필요 시 별도 승인된 sweep으로 확장한다.
6. 전체 path가 연결된 경우에만 GRASP task-local finger-target ACM과 attachment lifecycle을 적용한다. global SRDF는 변경하지 않는다.
7. dense validation: revolute step ≤0.01 rad, prismatic step ≤0.005 m; exact bounds, self collision, robot-box, premature target contact, attached-object-box collision을 전 state에서 검사한다.
8. raw trajectory, rejected point/joint/excess, IK seed, stage/waypoint, collision pair, clearance를 모두 저장한다.
9. 최소 5개의 dense-valid non-boundary 후보를 얻은 뒤 active-joint margin 최대 → environment/self clearance 최대 → path length 최소 → planning time 최소 순으로 reference를 선택한다. 이 lexicographic rule은 **PROVISIONAL**이며 실험 protocol에서 확정해야 한다.

## 14. 이후 stage-aware local recovery와 Lift/Yaw/Pitch 선택

**아직 구현되지 않았으며 아래는 PROVISIONAL DESIGN이다.**

1. reference의 각 state에 stage와 normalized progress를 부여한다.
2. nominal arm/reference follower가 실패하면 최초 실패 stage, waypoint, failure class를 기록한다.
3. 전체 문제를 다시 풀지 않고 실패 전후의 local window만 재계획하며, 성공 후 reference suffix에 재합류시킨다.
4. 후보 자유도는 작은 subset부터 순차적으로 연다: arm-only → Lift → Lift+Yaw → Lift+Pitch → Lift+Yaw+Pitch. exact URDF range 밖 후보는 제외하고 절대 clamp하지 않는다.
5. stage/failure 정보로 우선순위를 정한다. 예: 수직 reach/높이 실패는 Lift 우선, 좌우 wall clearance는 Yaw 후보 우선, approach/descent orientation/reach는 Pitch 후보를 평가한다. 이 mapping은 반드시 실험으로 검증하며 규칙을 성공 결과에 맞춰 사후 변경하지 않는다.
6. 각 candidate는 local segment 성공뿐 아니라 reference 재합류점부터 task suffix 전체를 dense validate한다.
7. 선택 기준 제안: full suffix success 필수 → violation 0 → minimum clearance 최대 → normalized torso displacement/variation 최소 → arm path length 최소 → planning time 최소.
8. normalized torso cost 예시는 각 joint 변화를 해당 joint range로 나눈 합이다. weight와 window size, candidate resolution은 현재 UNVERIFIED이며 최종 protocol 전에 고정해야 한다.

## 15. ICRA 최종 실험 계획과 측정 항목

### 비교군 제안

- Global/non-stage-aware arm-only 또는 LIFT_ONLY OMPL baseline.
- Global Lift–Yaw–Pitch OMPL baseline.
- Reference only, recovery 없음.
- Reference + Lift-only local recovery.
- Reference + Lift+Yaw.
- Reference + Lift+Pitch.
- Reference + full Lift+Yaw+Pitch stage-aware recovery.
- 선택 규칙 제거 ablation: failure class를 무시하고 모든 torso DOF를 항상 여는 방식.

### 장면/반복

- 고정 중앙 50 mm cube는 reference 생성과 sanity check에 사용한다.
- interior의 다양한 XYZ/corner/near-wall target set은 아직 정의되지 않았다: **UNVERIFIED**.
- 각 condition의 반복 수와 OMPL seed 통제 방식도 **UNVERIFIED**. Humble MoveGroupInterface에서 per-request OMPL seed가 노출되지 않았으므로 process-level repeat와 명시적 IK seed를 구분해야 한다.
- 기존 20-repeat pilot threshold는 방법 개발용이며 최종 protocol로 자동 승격하지 않는다.

### 필수 측정 항목

- full-task planning success rate와 95% confidence interval.
- nominal failure 중 recovery 성공률.
- 최초 실패 stage/waypoint와 failure taxonomy.
- end-to-end planning time, recovery latency, OMPL/IK 호출 수, candidates tested.
- joint-space path length와 TCP path length.
- Lift/Yaw/Pitch total motion, peak deviation, normalized torso cost, 선택된 DOF subset.
- minimum environment/self/attached-object clearance.
- minimum active-joint margin; exact joint-limit violation count와 excess.
- endpoint position/orientation error.
- premature finger-target, non-finger-target, robot-box, attached-object-box collision count.
- reference rejoin error와 rejoin 이후 suffix 성공 여부.
- stage별 success/failure 분해 및 target-location별 heatmap.
- random/process repeat reproducibility와 result manifest/hash.

현재 증거는 simulation/planning-only다. Hardware grasp success, execution time, force/torque, object slip은 hardware protocol이 별도로 승인되기 전에는 ICRA 결과 항목으로 주장하면 안 된다.

## 16. 현재 디렉터리 구조

```text
/home/openarm/humanoid_sim_ws                    (337 MB total, Git repo 아님)
├── ICRA_PROJECT_HANDOFF.md
├── SOURCE_PROVENANCE.md
├── run_humanoid_moveit.sh
├── run_humanoid_workspace_boundary.sh
├── src/
│   ├── humanoid_description/                    (이전/provisional description 및 검증)
│   ├── humanoid_sim_description/                (활성 AMR/Lift/Yaw/Pitch/OpenArm Xacro, 38 MB)
│   ├── humanoid_sim_moveit_config/              (SRDF/ACM/KDL/OMPL/planning-only, 140 KB)
│   ├── humanoid_extraction_experiments/         (실험 C++/launch/config, 752 KB)
│   └── openarm_description/                     (copied OpenArm Xacro/config/mesh, 147 MB)
├── validation/                                  (보고서, CSV, URDF, logs, backups)
│   ├── pilot_experiment_20260812/
│   ├── front_open_development_reference_20260812/
│   ├── orientation_candidates/
│   └── backups/
├── build/                                       (generated; Git 제외)
├── install/                                     (generated; Git 제외)
└── log/                                         (generated/personal paths; Git 제외)
```

## 17. 다음 Codex가 즉시 수행할 첫 번째 작업

**첫 작업: top-open stage-constrained reference planner v2를 별도 파일로 구현하기 전에, 현재 v1/grasp-seeded 실패를 재현 가능한 regression fixture로 고정한다.**

구체적으로:

1. 이 문서의 보호 파일 hash를 재확인한다.
2. 기존 결과를 덮어쓰지 않는 새 run directory/output prefix를 만든다.
3. v1의 Lift 0.30 forward 85.2941% 실패와 Lift 0.35 reverse 91.6667%/약 14 mm gap을 자동 판정하는 regression test/report를 만든다.
4. 그 다음 `stage_constrained_reference_generator_v2.cpp`에서 bidirectional constrained continuation/branch connection을 구현한다.
5. 성공 기준은 “한 segment가 더 길어짐”이 아니라 five-stage full path + dense validation + raw trajectory persistence다.

로봇 Xacro, SRDF/ACM, joint limits, meshes, fixed grasp, box geometry를 바꾸어 실패를 숨기지 말 것.

## 18. Git/cloud/GitHub 읽기 전용 감사

### Git 상태

- `/home/openarm/humanoid_sim_ws`: Git repository 아님.
- `/home/openarm`: Git repository 아님.
- workspace 아래 nested `.git` directory 없음.
- 따라서 current branch: **N/A**.
- configured remote: **N/A**.
- uncommitted files: **판정 불가/N/A**. 모든 파일이 아직 어떤 Git index에도 속하지 않는다.
- 이 감사 중 commit, push, repository 생성, 파일 삭제를 하지 않았다.

### 대용량/CAD/mesh

- workspace 총 크기 약 337 MB.
- 90 MB 초과 단일 파일 없음; GitHub 단일-file 100 MB 제한을 넘는 파일은 현재 발견되지 않았다.
- 가장 큰 source files:
  - `src/humanoid_sim_description/meshes/amr_base.stl` 20,506,384 bytes.
  - `.../lift_fixed_structure.stl` 13,917,584 bytes.
  - OpenArm `body_link0.dae` 각각 10,783,948 bytes.
  - OpenArm v1.0 visual `link5.dae` 9,034,473 bytes.
- `src/humanoid_sim_description` 38 MB, `src/openarm_description` 147 MB.
- `.iam/.ipt/.step/.stp` 원본 CAD는 workspace 안에서 발견되지 않았다. 다만 validation README에 Windows OneDrive IAM 원본 경로와 hash가 기록돼 있다.
- Mesh redistribution 권리와 CAD 공개 가능 여부는 **UNVERIFIED**. 큰 mesh는 Git LFS 검토 대상이다.

### 민감정보/개인 경로

- 파일명 기반 `.env`, PEM/private key, SSH key, credential/secret/token 파일은 발견되지 않았다.
- 일반적인 private-key/GitHub token/AWS key/password/API-key pattern의 내용 검색에서는 일치 파일이 없었다.
- 이는 비밀정보 부재를 보증하지 않는 정적 pattern scan이다.
- source 25개와 validation 66개 파일에 `/home/openarm`, `C:\Users\...\OneDrive`, 사용자명/hostname 또는 ROS log path 흔적이 있다.
- 특히 launch 파일의 absolute output path, validation logs/Markdown, `SOURCE_PROVENANCE.md`, `src/humanoid_sim_description/validation/README_KO.md`를 sanitize해야 한다.

### GitHub에 그대로 올리면 안 되는 범위

- `build/**`, `install/**`, `log/**`.
- `**/__pycache__/**`, `*.pyc`, object/binary build artifacts.
- `/home/openarm/.ros/**` 외부 ROS logs와 이를 복사한 host-specific logs.
- 개인 Windows/OneDrive 경로, hostname, absolute home path가 남은 validation/log/source 설정.
- `validation/backups/**` 전체를 무비판적으로 공개하는 것: 중복·과거 provisional 값·개인 경로가 포함된다.
- humanoid CAD-derived STL과 OpenArm copied meshes: 소유권/redistribution license 확인 전 공개 금지.
- `src/openarm_description`: package.xml은 Apache-2.0을 선언하지만 workspace copy에 top-level LICENSE/NOTICE가 없다. upstream license와 mesh asset license를 포함·확인하기 전 공개 금지.
- RViz screenshots/logs는 화면·hostname·환경정보가 없는지 수동 확인 후 선택적으로 공개.

### 안전하게 올릴 수 있는 범위

아래도 **경로 sanitize, license 보완, secret 재검사 후**에만 안전하다고 본다.

- 자체 작성 코드: `src/humanoid_extraction_experiments/{CMakeLists.txt,package.xml,src,scripts,launch,config,rviz}`.
- 자체 MoveIt 구성: `src/humanoid_sim_moveit_config/**` 중 generated artifact가 아닌 source/config/launch/rviz.
- 자체 humanoid Xacro/config/launch 문서. 단 CAD mesh는 별도 권리 확인 또는 private/LFS 처리.
- root helper scripts, 이 handoff, provenance 문서의 sanitized version.
- 최종 결과를 재현하는 curated `validation/*.md`, small YAML/CSV, hash manifest. raw multi-MB CSV와 host logs는 release artifact/object storage를 고려한다.
- 먼저 `.gitignore`, `.gitattributes`, LICENSE/NOTICE, data/mesh provenance policy를 만든 뒤 Git repository를 초기화해야 한다.

## 19. 근거 파일 목록

핵심 수치와 결론에 직접 사용한 파일:

- `SOURCE_PROVENANCE.md`
- `src/humanoid_sim_description/urdf/humanoid_sim.urdf.xacro`
- `src/humanoid_sim_description/urdf/{amr_base,lift,waist,openarm_arms_adapter}.xacro`
- `src/humanoid_sim_description/config/{cad_frame_transforms,validated_arm_mount_alignment,provisional_arm_mount_alignment,provisional_dynamics}.yaml`
- `src/humanoid_sim_description/validation/README_KO.md`
- `src/humanoid_description/validation/measured_joint_limits_validation.md`
- `src/openarm_description/assets/robot/openarm_v1.0/config/arm/joint_limits.yaml`
- `src/openarm_description/assets/end_effector/parallel_link/config/joint_limits.yaml`
- `validation/validated_mount_transition_after.urdf`
- `src/humanoid_sim_moveit_config/README.md`
- `src/humanoid_sim_moveit_config/config/{humanoid_sim.srdf,joint_limits.yaml,kinematics.yaml,ompl_planning.yaml,ompl_planning_fine.yaml,trajectory_validation.yaml}`
- `src/humanoid_sim_moveit_config/launch/planning_only.launch.py`
- `validation/{moveit_implementation_validation,validated_mount_transition,post_acm_collision_check,single_case_moveit_audit,joint_limit_violation_audit}.md`
- `src/humanoid_extraction_experiments/config/{confined_scene,top_open_box_600x400x150,top_open_reference_scene}.yaml`
- `validation/{grasp_orientation_audit,grasp_geometry_audit,object_lifecycle_verification,extraction_endpoint_audit,single_case_extraction_summary}.md`
- `validation/pilot_experiment_20260812/{pilot_conclusion,differential_target_analysis,differential_repeat_exit_audit}.md`
- `validation/pilot_experiment_20260812/{lift_only_vs_lift_yaw_pitch,differential_target_repeat_trials,planning_budget_sensitivity}.csv`
- `validation/{top_open_box_geometry_audit,top_open_box_visibility_audit,top_open_50mm_object_geometry_audit,top_entry_orientation_audit}.md`
- `validation/{top_open_center_reachability_audit,top_open_center_reachability_grasp_corrected}.md`
- `validation/{50mm_grasp_height_geometry_audit,reference_grasp_geometry_audit,reference_grasp_fine_height_audit}.md`
- `validation/reference_grasp_50mm.yaml`
- `validation/{reference_path_audit,top_open_reference_path_audit,stage_constrained_reference_audit,grasp_seeded_reference_audit}.md`
- 해당 `reference_*`, `top_open_reference_*`, `stage_constrained_*`, `grasp_seeded_*` CSV/YAML status artifacts.
- `src/humanoid_extraction_experiments/src/{reference_trajectory_generator,stage_constrained_reference_generator,grasp_seeded_reference_generator,differential_repeat_trials}.cpp`
- `src/humanoid_extraction_experiments/CMakeLists.txt`와 관련 launch files.

## 20. 인계 시 금지사항

- 아직 commit/push/GitHub repo 생성/파일 삭제를 하지 말 것.
- top-open 결과와 HISTORICAL front-open reference를 혼합하지 말 것.
- 실패를 해결하려고 Xacro/SRDF/ACM/joint limits/mesh/grasp/box를 임의 변경하지 말 것.
- exact limit 초과값을 clamp하지 말 것.
- `NOT_SELECTED` 2-line CSV를 실제 trajectory로 읽지 말 것.
- kinematic grasp를 force-closure 또는 hardware grasp로 주장하지 말 것.
- pilot 결과를 ICRA 최종 결과로 보고하지 말 것.

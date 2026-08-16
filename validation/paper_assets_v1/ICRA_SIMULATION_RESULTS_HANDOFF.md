# ICRA Simulation Results Handoff

## 1. 문서 목적과 해석 범위

이 문서는 완료된 planning-only simulation 결과를 ICRA 논문의 Simulation Results 절과 후속 실제 하드웨어 검증 계획으로 전달하기 위한 handoff이다. 수치와 분류는 저장된 `all_case_results.csv`, summary CSV, `paper_assets_v1` 표 및 기존 audit에서만 가져왔다. 이 문서를 만들기 위해 simulation, IK, OMPL, collision-distance 계산 또는 RViz replay를 다시 실행하지 않았다.

핵심 해석 단위는 **task-conditioned torso-axis activation**이다. 수직 접근과 인출은 Lift가 담당하고, Arm은 파지 자세 생성과 목표에 대한 수평 정렬을 담당한다. Yaw와 Pitch는 파지 전에만 선택하여 LOCKED 자세에서 불가능한 grasp configuration을 복구하며, 파지 후에는 선택된 torso 자세와 Arm을 고정하고 Lift만 이동한다. 따라서 본 결과는 수직 이동을 Arm IK에 맡기는 방식이나 인출 중 동적으로 몸통을 재조정하는 방식을 평가하지 않는다.

## 2. 연구 문제

연구 질문은 다음과 같다.

> Lift-only 수직 하강·상승 구조를 유지하면서, 작업 목표에 따라 Yaw와 Pitch 축을 선택적으로 활성화하면 LOCKED torso에서 존재하지 않거나 충돌로 배제되는 파지 자세를 복구하고, 조사된 표본에서의 feasible grasp workspace와 Arm joint-limit margin을 확장할 수 있는가?

역할 분담은 다음과 같이 고정했다.

- **Lift:** `lift_joint`만으로 0.17 m 하강하고 0.17 m 상승한다. `q_lift`가 증가하면 TCP가 world Z 아래쪽으로 이동하며, 감사된 `+0.001 m` Lift 변화에 대한 TCP 변화는 `(0, 0, -0.001) m`이다.
- **Arm:** top-grasp pose와 목표 X/Y 정렬을 위한 grasp configuration만 생성한다. Lift 하강·상승 중 Arm 최대 변화량은 0이다.
- **Yaw/Pitch:** grasp 전에 posture-selection 변수로만 사용한다. 선택 후 Lift 경로 전체에서 고정한다.
- **복구의 의미:** 동일 target, Lift, box/object, grasp orientation, collision model 및 deterministic search policy에서 LOCKED가 실패하고 단일 torso 축 또는 두 축을 허용한 모드가 성공하는 경우이다.

## 3. 실험 조건

### 3.1 장면과 물체

| 항목 | 저장된 조건 |
|---|---|
| 상부 개방 박스 내부 크기 | X/Y/Z = `0.400 × 0.600 × 0.150 m` (`600 × 400 × 150 mm` 표기) |
| 박스 내부 X 범위 | `[0.475, 0.875] m` |
| 박스 내부 Y 범위 | `[-0.100, 0.500] m` |
| 박스 내부 Z 범위 | `[0.940, 1.090] m` |
| 박스 상단 open plane | world Z = `1.090 m` |
| 물체 | `0.050 × 0.050 × 0.050 m` cube |
| 중앙 물체 중심 | `(0.675, 0.200, 0.965) m` |
| 파지 방향 | 기존 승인된 고정 top-grasp orientation |
| 추출 완료 조건 | 부착 물체가 박스 상단보다 `0.020 m` 높게 도달 |

박스는 floor/front/back/left/right의 다섯 collision object로 구성되며 top wall은 없다. `GRIPPER_ENVELOPE_INFEASIBLE` 판정은 50 mm 물체 AABB만이 아니라 link7과 양 finger의 실제 collision geometry 및 open-to-contact swept envelope를 사용한 기존 감사 분류를 유지한다.

### 3.2 관절 범위와 평가 Lift

| 변수 | 범위 또는 평가값 |
|---|---|
| Lift URDF 범위 | `[0.0, 0.7] m` |
| Phase-1 grasp Lift | `0.35 m`, `0.40 m` |
| Yaw URDF/탐색 범위 | `[-0.174533, +0.174533] rad` = `[-10°, +10°]` |
| Pitch URDF/탐색 범위 | `[-0.174533, +0.785398] rad` = `[-10°, +45°]` |
| Lift-only 하강/상승 | `0.17 / 0.17 m`, 최대 1 mm 간격의 저장된 검증 정의 |

요청 범위 끝점은 실제 URDF bound에 clamp되었다. joint limit, collision model, box/object geometry 및 grasp orientation은 모드 사이에서 동일하다.

### 3.3 네 가지 ablation 모드

| 모드 | Yaw | Pitch | 파지 후 동작 |
|---|---:|---:|---|
| `LOCKED` | 0 | 0 | Arm/torso 고정, Lift-only |
| `YAW_ONLY` | 활성 | 0 | 선택 Yaw와 Arm 고정, Lift-only |
| `PITCH_ONLY` | 0 | 활성 | 선택 Pitch와 Arm 고정, Lift-only |
| `YAW_PITCH` | 활성 | 활성 | 선택 Yaw/Pitch와 Arm 고정, Lift-only |

각 모드는 동일한 deterministic Arm IK seed 정책과 동일한 후보 평가 조건을 사용했다. 본 결과의 `YAW_PITCH`는 feasible candidate set이 단일 축 최적 후보를 포함하도록 dominance audit을 통과했으며, 저장된 931개 비교에서 dominance violation은 0개였다.

### 3.4 16-ray 평가

Phase 1은 box CENTER에서 22.5° 간격으로 배치된 16개 ray를 사용했다. 각 ray, Lift 및 모드에 대해 저장된 마지막 성공 거리 `d`를 `(ΔX, ΔY) = (d cos θ, d sin θ)`로 변환하고, 각도의 순서대로 16개 끝점을 닫아 polygon area를 계산했다.

- 평가 단위: 16 rays × 2 Lift values × 4 modes = 128 boundary rows.
- 각 모드의 경계는 독립적으로 결정되었다.
- 면적은 **16-ray polygon estimate**이며 exact continuous workspace가 아니다.
- Phase 2와 Phase 3는 이 Phase-1 면적 및 경계 그림에서 제외됐다.

### 3.5 Planning-only 범위

- 실제 하드웨어, controller, `ros2_control`, trajectory execution 및 force-closure 실험을 수행하지 않았다.
- Arm/torso 자세 선택 뒤에는 Lift만 움직이는 kinematic extraction을 검사했다.
- 성공 결과는 물리적 반복 성공률이나 실제 하드웨어 성능을 뜻하지 않는다.
- deterministic seed-bank 반복은 IK search sensitivity이며 물리적 repeatability가 아니다.

## 4. 성공과 실패 판정

### 4.1 성공

`LIFT_ACTUATED_EXTRACTION_SUCCESS` / `NOMINAL_FEASIBLE`은 다음 저장 조건을 만족한 경우다.

1. joint limit 안에서 collision-free grasp configuration을 선택한다.
2. Arm과 선택된 Yaw/Pitch를 고정한다.
3. Lift-only로 0.17 m 하강하고 planning/kinematic grasp 상태를 만든다.
4. 물체를 부착한 뒤 Lift-only로 0.17 m 상승한다.
5. self/environment collision이 없고, Arm 최대 변화량이 0이며, 부착 물체가 box top보다 0.020 m 위에 도달한다.

이는 force closure, grasp force 또는 실제 pick 성공을 의미하지 않는다.

### 4.2 실패 및 주의 분류

- `PURE_IK_ABSENCE`: 고정된 target/torso 조건에서 grasp configuration IK가 없다. 같은 target에서 torso 축 활성화로 성공하면 posture-conditioned recovery로 볼 수 있다.
- `GRASP_CONFIGURATION_COLLISION`: raw IK가 있더라도 collision-free grasp candidate가 남지 않는 경우다. 저장된 해당 모드의 실패이며 자동으로 torso-recoverable이라고 해석하지 않는다.
- `GRIPPER_ENVELOPE_INFEASIBLE`: 실제 gripper/finger swept geometry가 벽과 필연적으로 겹치는 기하학적 불가능이다. torso failure 및 torso recovery 실패 통계에서 분리해야 한다.
- `simulation nominal low-clearance`: 수치상 collision-free 성공이지만 최소 clearance가 매우 작아 hardware validation 후보에서 제외해야 하는 조건이다. Phase-1 최소 자기충돌 clearance는 `0.043687139 mm`였다.

Phase-1 저장 실패 분포는 다음과 같다.

| 모드 | PURE_IK_ABSENCE | GRASP_CONFIGURATION_COLLISION | GRIPPER_ENVELOPE_INFEASIBLE |
|---|---:|---:|---:|
| LOCKED | 42 | 0 | 22 |
| YAW_ONLY | 215 | 0 | 26 |
| PITCH_ONLY | 0 | 120 | 26 |
| YAW_PITCH | 0 | 34 | 26 |

이 count는 저장된 Phase-1 failed case row의 분류이며 전체 작업 성공률이 아니다.

## 5. 동일 목표 복구 결과

### 5.1 Yaw-only 동일 목표 복구

두 행의 phase, target XYZ, ray, ray distance, Lift, seed bank, box/object, grasp orientation, collision 조건 및 Lift motion은 동일하다. 유일한 실험 독립변수는 Yaw 활성화 여부이며 Pitch는 두 모드 모두 0으로 고정됐다.

| 항목 | LOCKED | YAW_ONLY |
|---|---|---|
| Unique key | `PHASE1\|R3_0.030\|R3\|0.030\|0.400\|LOCKED\|0` | `PHASE1\|R3_0.030\|R3\|0.030\|0.400\|YAW_ONLY\|0` |
| Target XYZ | `(0.686480503, 0.227716386, 0.965) m` | 동일 |
| Ray / distance / Lift | `R3 / 0.030 m / 0.40 m` | 동일 |
| Yaw / Pitch | `0° / 0°` | `−8° / 0°` |
| 결과 | `GRASP_CONFIGURATION_IK_FAILURE`, `PURE_IK_ABSENCE` | `LIFT_ACTUATED_EXTRACTION_SUCCESS`, `NOMINAL_FEASIBLE` |
| Arm minimum margin | 정의되지 않음: 저장 Arm posture 없음 | `0.0528147 rad` |
| Environment / self clearance | 정의되지 않음 | `2.53962 / 0.65926 mm` |

LOCKED 실패 row에는 성공 trajectory나 선택 Arm posture가 없으므로 margin/clearance를 0으로 치환하지 않는다.

### 5.2 Pitch-only 동일 목표 복구

두 행의 phase, target XYZ, ray, ray distance, Lift, seed bank, box/object, grasp orientation, collision 조건 및 Lift motion은 동일하다. 유일한 실험 독립변수는 Pitch 활성화 여부이며 Yaw는 두 모드 모두 0으로 고정됐다.

| 항목 | LOCKED | PITCH_ONLY |
|---|---|---|
| Unique key | `PHASE1\|R3_0.020\|R3\|0.020\|0.350\|LOCKED\|0` | `PHASE1\|R3_0.020\|R3\|0.020\|0.350\|PITCH_ONLY\|0` |
| Target XYZ | `(0.682653669, 0.218477591, 0.965) m` | 동일 |
| Ray / distance / Lift | `R3 / 0.020 m / 0.35 m` | 동일 |
| Yaw / Pitch | `0° / 0°` | `0° / +18°` |
| 결과 | `GRASP_CONFIGURATION_IK_FAILURE`, `PURE_IK_ABSENCE` | `LIFT_ACTUATED_EXTRACTION_SUCCESS`, `NOMINAL_FEASIBLE` |
| Arm minimum margin | 정의되지 않음: 저장 Arm posture 없음 | `0.436902 rad` |
| Environment / self clearance | 정의되지 않음 | `2.53961 / 0.65926 mm` |

연결 자산:

- Figure: [`figures/figure1_same_target_single_axis_recovery_2x2.pdf`](figures/figure1_same_target_single_axis_recovery_2x2.pdf), [`PNG`](figures/figure1_same_target_single_axis_recovery_2x2.png)
- Table: [`tables/table2_same_target_axis_ablation.csv`](tables/table2_same_target_axis_ablation.csv), [`Markdown`](tables/table2_same_target_axis_ablation.md)

이 두 사례는 Yaw와 Pitch가 각각 독립적으로 LOCKED 실패를 복구한 사례다. 두 단일 축이 모두 실패하고 결합 모드만 성공한 비교가 아니므로 Yaw–Pitch synergy나 두 축의 동시 필요성을 보이지 않는다.

## 6. 16-ray 작업영역 결과

| Lift (m) | 모드 | Estimated polygon area (m²) | LOCKED 대비 증가율 |
|---:|---|---:|---:|
| 0.35 | LOCKED | 0.0599766 | 0% |
| 0.35 | YAW_ONLY | 0.0744557 | 24.1411% |
| 0.35 | PITCH_ONLY | 0.0948205 | 58.0958% |
| 0.35 | YAW_PITCH | 0.100853 | 68.1541% |
| 0.40 | LOCKED | 0.0633906 | 0% |
| 0.40 | YAW_ONLY | 0.0777284 | 22.6182% |
| 0.40 | PITCH_ONLY | 0.0964156 | 52.0977% |
| 0.40 | YAW_PITCH | 0.102598 | 61.8512% |

연결 자산:

- Boundary figure: [`figures/figure2_workspace_boundary_16ray_estimate.pdf`](figures/figure2_workspace_boundary_16ray_estimate.pdf), [`PNG`](figures/figure2_workspace_boundary_16ray_estimate.png)
- Area figure: [`figures/figure3_workspace_area_comparison.pdf`](figures/figure3_workspace_area_comparison.pdf), [`PNG`](figures/figure3_workspace_area_comparison.png)
- Table: [`tables/table1_lift_16ray_estimated_area.csv`](tables/table1_lift_16ray_estimated_area.csv), [`Markdown`](tables/table1_lift_16ray_estimated_area.md)

해석은 “조사된 16-ray 표본에서 torso-axis activation이 LOCKED 대비 estimated polygon area를 확장했다”로 제한한다. `YAW_PITCH` 대 `LOCKED` 증가율은 결합 모드의 직접 비교이며 synergy 지표가 아니다.

## 7. 관절 여유와 충돌 여유

다음 값은 finite margin/clearance가 저장된 Phase-1 feasible 표본만 집계한 것이다. IK-absence 실패 row는 margin 0으로 포함하지 않았다.

| 모드 | finite n | Arm margin min / mean / max (rad) | Environment clearance min / mean / max (mm) | Self clearance min / mean / max (mm) |
|---|---:|---:|---:|---:|
| LOCKED | 479 | 0 / 0.114457 / 0.656478 | 0.428247 / 2.44883 / 2.54462 | 0.65926 / 0.65926 / 0.65926 |
| YAW_ONLY | 571 | 0.0007742 / 0.132469 / 0.687359 | 0.429060 / 2.46065 / 2.54543 | 0.65926 / 0.65926 / 0.65926 |
| PITCH_ONLY | 656 | 0.0232380 / 0.285886 / 0.696979 | 0.429073 / 2.46844 / 2.54554 | 0.65926 / 0.65926 / 0.65926 |
| YAW_PITCH | 679 | 0.0790753 / 0.323835 / 0.712503 | 0.429073 / 2.47084 / 2.54404 | **0.0436871** / 0.622387 / 0.65926 |

저장된 finite feasible 표본 범위에서는 PITCH_ONLY와 YAW_PITCH가 YAW_ONLY보다 평균 Arm margin이 컸다. 이 결과로 Pitch가 조사된 task posture selection에서 Arm limit margin 개선에 더 유리했다고 기술할 수 있지만, 실제 하드웨어 강건성이나 모든 작업에서의 우월성을 뜻하지 않는다.

`0.0436871 mm` YAW_PITCH 자기충돌 clearance는 collision-free numerical success이지만 **simulation nominal low-clearance**로 분류한다. 논문에서는 nominal success count와 hardware-candidate count를 분리하고, 이 조건을 실제 하드웨어 실행 후보에서 제외해야 한다.

연결 자산:

- Figure: [`figures/figure4_arm_joint_margin_comparison.pdf`](figures/figure4_arm_joint_margin_comparison.pdf), [`PNG`](figures/figure4_arm_joint_margin_comparison.png)
- Table: [`tables/table3_joint_margin_collision_clearance.csv`](tables/table3_joint_margin_collision_clearance.csv), [`Markdown`](tables/table3_joint_margin_collision_clearance.md)

## 8. Margin-threshold usable-workspace sensitivity

아래 면적은 Arm minimum margin이 nominal `> 0`, 1°, 2°, 5°, 10°를 넘는 stored success에 대한 simulation sensitivity criterion이다. 실제 하드웨어 tolerance가 아니다.

### Lift 0.35 m

| Threshold | LOCKED | YAW_ONLY | PITCH_ONLY | YAW_PITCH |
|---|---:|---:|---:|---:|
| nominal | 0.048855 | 0.074456 | 0.094821 | 0.100853 |
| 1° | 0.038556 | 0.058167 | 0.094821 | 0.100853 |
| 2° | 0.037190 | 0.049929 | 0.094060 | 0.100853 |
| 5° | 0.028538 | 0.038633 | 0.088991 | 0.100585 |
| 10° | 0.027557 | 0.028573 | 0.082584 | 0.089297 |

### Lift 0.40 m

| Threshold | LOCKED | YAW_ONLY | PITCH_ONLY | YAW_PITCH |
|---|---:|---:|---:|---:|
| nominal | 0.059230 | 0.077728 | 0.096416 | 0.102598 |
| 1° | 0.050716 | 0.067234 | 0.096416 | 0.102598 |
| 2° | 0.049372 | 0.065332 | 0.095365 | 0.102598 |
| 5° | 0.047548 | 0.055732 | 0.091182 | 0.101984 |
| 10° | 0.035874 | 0.035299 | 0.083261 | 0.087547 |

단위는 모두 `m²`의 16-ray polygon estimate다. threshold가 커질수록 LOCKED와 YAW_ONLY의 usable estimate가 크게 감소하고, PITCH_ONLY와 YAW_PITCH는 조사된 표본에서 더 큰 면적을 유지했다. 이는 Arm joint-limit margin에 대한 simulation sensitivity이며 실제 backlash, calibration error 또는 compliance를 모델링한 결과가 아니다.

연결 자산: [`figures/figure5_robustness_and_failure_taxonomy.pdf`](figures/figure5_robustness_and_failure_taxonomy.pdf), [`PNG`](figures/figure5_robustness_and_failure_taxonomy.png).

## 9. 논문에서 가능한 주장과 불가능한 주장

### 가능한 주장

1. 조사된 planning-only 16-ray 표본에서 task-conditioned torso-axis activation은 LOCKED 대비 grasp feasibility와 estimated polygon workspace를 확장했다.
2. 저장된 finite feasible 표본에서 PITCH_ONLY와 YAW_PITCH의 평균 Arm margin은 YAW_ONLY보다 컸으며, Pitch activation이 이 task에서 Arm joint-limit margin 개선에 더 유리한 경향을 보였다.
3. 동일 target과 Lift에서 다른 조건을 고정했을 때, Yaw-only `−8°`가 한 LOCKED `PURE_IK_ABSENCE`를 복구한 사례가 있다.
4. 동일 target과 Lift에서 다른 조건을 고정했을 때, Pitch-only `+18°`가 별도의 LOCKED `PURE_IK_ABSENCE`를 복구한 사례가 있다.
5. gripper-envelope 기하학적 불가능은 torso posture recovery 실패와 분리해야 한다.

### 불가능한 주장

1. 16-ray polygon을 exact continuous workspace라고 주장할 수 없다.
2. 현재 동일 목표 비교로 Yaw–Pitch synergy 또는 두 축의 동시 필요성을 주장할 수 없다.
3. simulation margin과 deterministic seed 결과로 hardware robustness 또는 물리적 repeatability를 주장할 수 없다.
4. planning/kinematic grasp로 force closure, grasp force 또는 접촉 안정성을 주장할 수 없다.
5. 저장된 표본의 success fraction을 실제 작업 성공률이라고 주장할 수 없다.
6. `0.0436871 mm` nominal low-clearance 사례를 별도의 실제 안전 검증 없이 hardware-feasible이라고 주장할 수 없다.

## 10. 실제 하드웨어 검증 계획

이 절은 향후 검증 프로토콜이며 현재 simulation 결과가 실제 하드웨어에서 확인됐다는 뜻이 아니다.

### 10.1 비교 조건

1. **Yaw 동일 목표 비교**
   - Target: `(0.686480503, 0.227716386, 0.965) m`
   - Lift: `0.40 m`
   - 비교: `LOCKED (0°, 0°)` 대 `YAW_ONLY (−8°, 0°)`
   - LOCKED에 실행 가능한 plan이 없으면 로봇을 움직이지 않고 planning-gate failure로 기록한다.
2. **Pitch 동일 목표 비교**
   - Target: `(0.682653669, 0.218477591, 0.965) m`
   - Lift: `0.35 m`
   - 비교: `LOCKED (0°, 0°)` 대 `PITCH_ONLY (0°, +18°)`
   - LOCKED에 실행 가능한 plan이 없으면 로봇을 움직이지 않고 planning-gate failure로 기록한다.
3. **공통 중앙 성공 목표**
   - Target: `(0.675, 0.200, 0.965) m`
   - 저장된 Phase-1에서 두 Lift와 네 모드 모두 nominal success row가 존재한다.
   - 실제 적용 전 아래 safety gate를 통과한 모드/Lift 조건만 실행한다.

각 유효 조건은 최소 20회 수행한다. 모드 비교 사이에는 target pose, Lift, grasp orientation, object/box geometry, controller gains, speed profile 및 안전 조건을 동일하게 유지한다.

### 10.2 사전 안전 gate

- simulation의 `0.0436871 mm` self-clearance를 포함한 nominal low-clearance target은 실제 실행 후보에서 제외한다.
- Arm joint margin, calibrated environment clearance 및 self-clearance에 대한 실제 하드웨어용 threshold는 calibration과 uncertainty budget으로 별도 확정한다. 본 simulation의 1°/2°/5°/10° sensitivity threshold를 hardware tolerance로 그대로 사용하지 않는다.
- joint limit, collision, controller readiness, emergency stop, payload attachment 및 target registration 검사를 통과하지 못한 trial은 실행하지 않고 실패/제외 이유를 보존한다.
- LOCKED IK-absence 조건에 대해 임의 자세나 trajectory를 만들어 실행하지 않는다.

### 10.3 측정 항목

각 조건과 반복에 대해 다음을 기록한다.

- planning-gate 통과 및 실제 pick/extraction 성공 여부
- planning 시간과 실제 수행시간
- 최종 TCP 및 물체 위치오차
- 전체 경로의 최소 Arm joint-limit margin
- 최소 environment/self clearance의 예측값과 관측 가능한 접촉/충돌 이벤트
- finger/object 접촉 여부, slip/drop 및 box-wall 접촉 여부
- safety abort 및 failure stage/label

성공률은 실행된 hardware trial의 분모와 planning-gate rejection을 구분해 보고한다. simulation result와 hardware result를 하나의 성공률로 합치지 않는다.

## 11. 논문용 Figure 및 Table 목록

| ID | 파일 | 본문에서 전달할 메시지 | 추천 캡션 초안 |
|---|---|---|---|
| Fig. 1 | `figure1_same_target_single_axis_recovery_2x2.pdf/.png` | 동일 목표에서 Yaw와 Pitch가 각각 LOCKED IK failure를 독립적으로 복구한 저장 replay 비교 | **Same-target single-axis posture recovery in planning-only simulation.** (a,c) With Yaw and Pitch locked, no grasp IK posture was stored for the respective target. (b) Activating Yaw alone at −8° and (d) activating Pitch alone at +18° produced collision-free Lift-only extraction while all task geometry and Lift conditions were held fixed. These examples do not establish Yaw–Pitch synergy. |
| Fig. 2 | `figure2_workspace_boundary_16ray_estimate.pdf/.png` | Lift별 네 모드의 방향별 도달 경계 형상 | **Planning-only feasible-boundary comparison at two Lift configurations.** Polygons connect the last successful samples on 16 rays and therefore estimate, rather than exactly represent, the continuous workspace. |
| Fig. 3 | `figure3_workspace_area_comparison.pdf/.png` | LOCKED 대비 단일 축 및 결합 posture selection의 estimated area 확장 | **Estimated 16-ray polygon areas for four torso-axis activation modes.** YAW_PITCH increased the sampled polygon estimate relative to LOCKED by 68.2% at Lift 0.35 m and 61.9% at Lift 0.40 m; these direct increases are not synergy measures. |
| Fig. 4 | `figure4_arm_joint_margin_comparison.pdf/.png` | finite feasible 표본에서 Pitch 포함 모드의 평균 Arm margin이 큼 | **Arm joint-limit margins among finite feasible Phase-1 samples.** Bars show means and error bars show stored minima and maxima; IK-absence rows are excluded rather than assigned zero margin. |
| Fig. 5 | `figure5_robustness_and_failure_taxonomy.pdf/.png` | margin threshold sensitivity, collision clearance 및 기하학적 실패 분리 | **Planning-only margin sensitivity and failure taxonomy.** Areas are 16-ray simulation sensitivity estimates, the 0.043687 mm self-clearance case is marked nominal low-clearance, and gripper-envelope infeasibility is separated from torso-posture failure. |
| Table 1 | `table1_lift_16ray_estimated_area.csv/.md` | Lift·모드별 estimated area와 LOCKED 대비 증가율 | **Estimated 16-ray polygon area by Lift and torso-axis activation mode.** |
| Table 2 | `table2_same_target_axis_ablation.csv/.md` | 정확히 동일한 target/Lift의 LOCKED 실패와 단일 축 성공 | **Same-target Yaw-only and Pitch-only recovery ablations under fixed task conditions.** |
| Table 3 | `table3_joint_margin_collision_clearance.csv/.md` | finite feasible sample의 Arm margin과 collision clearance | **Arm joint-limit margin and collision-clearance statistics for stored feasible Phase-1 samples.** |

## 12. 사용한 입력 파일과 SHA-256

| 입력 파일 | SHA-256 |
|---|---|
| `validation/paper_assets_v1/paper_assets_audit.md` | `7db694f1f55604457b90994a3d0607b1088f9a87fa8d4ce8403a5a27d0faa2c2` |
| `validation/paper_assets_v1/tables/table1_lift_16ray_estimated_area.csv` | `c334bfc257bc6f75fd8a0187597f59d9ecca93c609fa3a4b18a86d8926f5648d` |
| `validation/paper_assets_v1/tables/table2_same_target_axis_ablation.csv` | `90bd7d376af7dd70a231a40e134a92f46527afb0fb6d7520ffdd4060c6e46b59` |
| `validation/paper_assets_v1/tables/table3_joint_margin_collision_clearance.csv` | `c2b56393cbabceedf34449dbbc5d20d75f88fce1ae550621e89178067db9ea61` |
| `validation/paper_main_simulation_dataset_v1/run_20260815_223216/all_case_results.csv` | `f95048fe3e48ebffd3d1f255242a22ea03f8505dd59aa19ae3658aef8b696cf8` |
| `validation/paper_main_simulation_dataset_v1/run_20260815_223216/summaries/dataset_audit.md` | `c3bb4dcb8c748a92fbf4c63bb63db3121e3fe5709484221559e13604528f16d0` |
| `validation/paper_main_simulation_dataset_v1/run_20260815_223216/summaries/margin_threshold_sensitivity.csv` | `b460b903508d0b11bb3731da1a29a464925bfb7a7928e98feefd4fcb6ff9db48` |
| `validation/paper_main_simulation_dataset_v1/run_20260815_223216/summaries/failure_taxonomy.csv` | `9b7ef83997e00792c22f53a7b4fbea09a31af5535c64edfe28e3b4d619e0400b` |
| `validation/adaptive_target_boundary_search_v1/run_20260815_201241/adaptive_boundary_audit.md` | `0699e9b67fa18132999c1f639d34aac752f769873bd4d1b5d277e5fddd55d36a` |
| `validation/box_target_yaw_pitch_feasibility_pilot_v1/run_20260815_192422/pilot_audit.md` | `29a731936c4599b00ca6d30e25382ee64607ea3c7193ec9347325880349e4d1c` |
| `validation/lift_actuated_extraction_baseline_v1/run_20260815_165520/lift_actuated_audit.md` | `383e684d7d8499c035babefedde3364330e2034459168818faf512570a68fba4` |
| `validation/top_open_box_geometry_audit.md` | `483ecfbff64539f3bdaf7114af1b42b02257a0637b80c49809394e72ce313339` |
| `validation/top_open_50mm_object_geometry_audit.md` | `aad67c93b753b88815af8c710f5a073bfaef1387cbdaac0a7bccb7041a23dee9` |
| `validation/single_case_moveit_audit.md` | `f7d51e6f888ef9f88541b86f7e5fd0698fdc9c1e2234c7b829efbd575246c792` |

### 연결한 paper asset 해시

| Asset | SHA-256 |
|---|---|
| `figures/figure1_same_target_single_axis_recovery_2x2.pdf` | `b6800a63e1cef828a211d0f7a07d96ea341dc9b071349e6a784651fde6181bcb` |
| `figures/figure1_same_target_single_axis_recovery_2x2.png` | `c44a27117d6628d857731171e0371b3fad2ec9f85269db6b313c84c0708b3d4d` |
| `figures/figure2_workspace_boundary_16ray_estimate.pdf` | `0c655a35c25f6c25e3d20c69e878a77fda977c5c7f1db7696470fca1c527dd7a` |
| `figures/figure2_workspace_boundary_16ray_estimate.png` | `a4344c916296340185efc750cfae69c439292794c88ce2a8194ac1c4ca92c71c` |
| `figures/figure3_workspace_area_comparison.pdf` | `ed5e4ab2276647c1ee2452ccef358d1bd7440f7ada3f000d36731667ce56bbcf` |
| `figures/figure3_workspace_area_comparison.png` | `147e32136cc3197289be7694f4bcd16462e516ac700a5ea4f0c8653fd43c6d3f` |
| `figures/figure4_arm_joint_margin_comparison.pdf` | `5aaf7d6b59311a94f6f67d4ac75af5b9aa7b19a054b01af1e32bcd7400bd8eb7` |
| `figures/figure4_arm_joint_margin_comparison.png` | `6abbea96f25e14a8d1dcc74fa7bfcf7d4d99a3213af2afe140bd7be4a2b3b7a9` |
| `figures/figure5_robustness_and_failure_taxonomy.pdf` | `0d449aa63dd1b47ef7b388de8b0615c0fba7ec0508233522d219bdac7daf3b8a` |
| `figures/figure5_robustness_and_failure_taxonomy.png` | `ac5208954374c0ba60a2e3f355245c40b7b97d946ee8837222311f5c02321a86` |

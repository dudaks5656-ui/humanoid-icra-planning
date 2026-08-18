# Fixed-base workspace RViz presentation demo

## 연구 목적

AMR을 고정한 상태에서 Waist Yaw/Pitch 자유도가 manipulation workspace에 주는 영향을 동일한 1,440개 TCP 점으로 보여주는 planning-only 발표 자료입니다.

## 네 configuration과 검증 수치

| Configuration | Reachable | Targeted volume | C0 대비 | Max X |
|---|---:|---:|---:|---:|
| C0 Arm + Lift | 833 / 1440 | 0.097791 m³ | baseline | 0.6771 m |
| C1 Arm + Lift + Yaw | 1030 / 1440 | 0.120918 m³ | +23.65% | 0.7063 m |
| C2 Arm + Lift + Pitch | 976 / 1440 | 0.114578 m³ | +17.17% | 0.7063 m |
| C3 Arm + Lift + Yaw + Pitch | 1119 / 1440 | 0.131366 m³ | +34.33% | 0.7354 m |

- Yaw unique expansion: 78 points
- Pitch unique expansion: 24 points
- Yaw/Pitch overlap expansion: 119 points
- Combined-torso-only: 65 points, 0.007631 m³

## 영상 장면

1. 고정 AMR과 휴머노이드 모델
2. C0 Arm+Lift workspace
3. C1 Yaw 추가 workspace
4. C2 Pitch 추가 workspace
5. C3 Yaw+Pitch 및 category overlay
6. 동일 combined-only TCP에서 C0/C1/C2 실패
7. C3 성공 RobotState와 collision-checked visualization-only interpolation

RViz는 RobotState, TF, TCP marker와 workspace point cloud를 표시합니다. 수치 패널은 RViz 장면 topic과 동기화되는 visualization-only 2D overlay이며 validated CSV만 읽습니다.

## 실행

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fixed_base_workspace_demo fixed_base_workspace_demo.launch.py demo_scene:=auto
```

수동 장면은 `demo_scene:=c0`, `c1`, `c2`, `c3`, `combined_only`, `robot`으로 선택합니다. 측면 시점은 `rviz_config:=.../fixed_base_workspace_demo_side.rviz`를 사용합니다.

## 발표 시 말할 핵심

1. AMR은 움직이지 않습니다.
2. 동일한 1,440 TCP points를 비교했습니다.
3. IK, joint limit, self-collision 조건이 적용되었습니다.
4. Yaw 단독은 +23.65%, Pitch 단독은 +17.17%입니다.
5. Yaw+Pitch는 +34.33%입니다.
6. 65점은 두 torso DOF를 동시에 사용해야 했습니다.
7. 애니메이션은 RobotState 시각화이며 실제 trajectory execution이 아닙니다.
8. 다음 단계는 box task-feasible workspace와 recovery planning입니다.

## 녹화 파일

- `fixed_base_workspace_demo.mp4`: 전체 발표 버전
- `fixed_base_workspace_demo_short.mp4`: 단축 버전
- `frame_c0.png`, `frame_c3.png`, `frame_combined_only.png`: 대표 화면

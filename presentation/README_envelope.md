# 3D Workspace Envelope Visualization

이 자료는 기존 fixed-base workspace/4-way Waist DOF ablation의 검증된 1,440개 TCP grid를 연속된 3D 작업영역으로 보여줍니다. 기존 실험이나 IK grid sampling을 다시 수행하지 않습니다.

## 재구성 방식

1. `fixed_base_workspace_dof_ablation_comparison.csv`의 X/Y/Z unique level에서 12×10×12 grid와 `dx/dy/dz`를 복원합니다.
2. C0~C3의 PASS point를 동일 중심을 갖는 occupied voxel로 해석합니다.
3. 각 occupied voxel의 6개 이웃을 검사합니다.
4. 이웃도 occupied이면 내부 면을 버리고, 이웃이 없거나 unreachable이면 exposed face만 두 triangle로 표시합니다.

Convex hull이나 smoothing을 사용하지 않습니다. 따라서 concave boundary와 unreachable hole은 검증 grid 그대로 보존됩니다. Surface는 새로운 workspace 추정값이 아니라 기존 voxel volume의 외곽 표현입니다.

## 표시 모드

- `visualization_mode:=surface` — exposed-face `TRIANGLE_LIST` (발표 기본값)
- `visualization_mode:=voxels` — occupied voxel `CUBE_LIST`
- `visualization_mode:=points` — 원본 reachable grid point

수동 scene은 `robot`, `c0_volume`, `c1_volume`, `c2_volume`, `c3_volume`, `c0_vs_c3`, `all_four`, `yaw_expansion`, `pitch_expansion`, `combined_only`, `auto`를 지원합니다.

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fixed_base_workspace_envelope_demo fixed_base_workspace_envelope_demo.launch.py \
  demo_scene:=auto visualization_mode:=surface
```

## 발표 핵심

- C0 Arm+Lift: 833/1440, 0.097791 m³
- C1 +Yaw: 1030/1440, 0.120918 m³ (+23.65%)
- C2 +Pitch: 976/1440, 0.114578 m³ (+17.17%)
- C3 +Yaw+Pitch: 1119/1440, 0.131366 m³ (+34.33%)
- Combined-torso-only: 65 voxels, 0.007631 m³
- Point 1360: TCP `(0.7354, 0.1470, 1.0313) m`, C0/C1/C2 FAIL → C3 PASS

로봇 동작은 `/display_robot_state`에 publish하는 self-collision-checked visualization-only interpolation입니다. 실제 trajectory execution, controller, `ros2_control`, hardware 또는 AMR motion은 사용하지 않습니다.

## 영상

- `fixed_base_workspace_envelope_demo.mp4` — 전체 자동 sequence
- `fixed_base_workspace_envelope_demo_short.mp4` — 단축 sequence
- `envelope_c0.png`, `envelope_c1.png`, `envelope_c2.png`, `envelope_c3.png`
- `envelope_c0_vs_c3.png`, `envelope_combined_only.png`

기존 발표 자료와 manifest를 보존하기 위해 기존 `presentation/README.md`는 변경하지 않았습니다.

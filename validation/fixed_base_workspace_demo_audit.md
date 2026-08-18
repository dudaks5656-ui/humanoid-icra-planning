# Fixed-base workspace presentation demo audit

## Immutable sources

- `/home/openarm/humanoid_sim_ws/validation/fixed_base_workspace_dof_ablation_comparison.csv` — `5d7f1ea256b99ee0a36b1e8eab9250b34d89bcf9dce3c9fc61671eccd10f96b2`
- `/home/openarm/humanoid_sim_ws/validation/fixed_base_workspace_dof_ablation_summary.csv` — `f4c35452f7d896831931441329f13b5fdd680c423d5f7648ec45784d97b6c330`
- `/home/openarm/humanoid_sim_ws/validation/fixed_base_workspace_dof_ablation_contributions.csv` — `e17a42791e5c384b686edc9bb2f254a20241ec6f1192260e937637ef4ae61a0c`
- `/home/openarm/humanoid_sim_ws/validation/fixed_base_workspace_dof_ablation_common_point_metrics.csv` — `11df09f202bd07f62c525ffd3d4c97be5fd6794cf9fa7749ad56e7cb3380126d`
- Existing manifest verification: PASS {'coarse': 14, 'fine': 21, 'ablation': 24}

## Point clouds and representative

- Point-cloud counts C0/C1/C2/C3: [833, 1030, 976, 1119]
- Combined-torso-only count: 65
- Representative point: ID `1360`, XYZ (`0.735416666666667`, `0.147`, `1.03125`) m
- Source classification/results: `COMBINED_TORSO_ONLY`, C0/C1/C2 FAIL, C3 PASS
- Confidence: `HIGH`
- Selected torso: Lift `0.55`, Yaw `0.116355333333333`, Pitch `0.425423875` rad
- Reproduced state margin/clearance: `0.0516381570975601` / `0.000659260018009228` m
- TCP position/orientation error: `6.24940836710841e-09` m / `0` rad
- Selected C3 joint state: `[('lift_joint', '0.55'), ('openarm_left_finger_joint1', '0.022'), ('openarm_left_finger_joint2', '0.022'), ('openarm_left_joint1', '0.80051156127777'), ('openarm_left_joint2', '-3.16106470020536'), ('openarm_left_joint3', '-0.594381163806068'), ('openarm_left_joint4', '0.467115887027492'), ('openarm_left_joint5', '0.541426243219617'), ('openarm_left_joint6', '-0.156686928051108'), ('openarm_left_joint7', '-1.51915784290244'), ('openarm_right_finger_joint1', '0.022'), ('openarm_right_finger_joint2', '0.022'), ('openarm_right_joint1', '0'), ('openarm_right_joint2', '0'), ('openarm_right_joint3', '0'), ('openarm_right_joint4', '0'), ('openarm_right_joint5', '0'), ('openarm_right_joint6', '0'), ('openarm_right_joint7', '0'), ('waist_pitch_joint', '0.425423875'), ('waist_yaw_joint', '0.116355333333333')]`

## Demo

- Scene sequence: ROBOT → C0 → C1 → C2 → C3 → C0/C1/C2 same-target failures → C3 same-target success → visualization interpolation → FINAL
- Animation collision-free: `True`, samples: `101`
- RViz configs: `fixed_base_workspace_demo.rviz`, `fixed_base_workspace_demo_side.rviz`

- Presentation text: synchronized 2D overlay; read-only CSV/status subscriber, no command publisher

## Recording

- Full command: `gst-launch-1.0 -e ximagesrc startx=140 starty=128 endx=2059 endy=1207 use-damage=false show-pointer=false ! videorate drop-only=true max-rate=30 ! video/x-raw,framerate=30/1 ! identity sync=true eos-after=1920 ! videoconvert ! videoscale add-borders=true ! video/x-raw,width=1920,height=1080,format=I420,pixel-aspect-ratio=1/1 ! x264enc bitrate=8000 speed-preset=medium key-int-max=60 threads=0 ! video/x-h264,stream-format=avc,alignment=au ! mp4mux faststart=true ! filesink location=/home/openarm/humanoid_sim_ws/presentation/fixed_base_workspace_demo.mp4`
- Full video: `/home/openarm/humanoid_sim_ws/presentation/fixed_base_workspace_demo.mp4`, 63.944 s, 1920×1080, 30 fps, H.264/MP4, 8806301 bytes
- Short video: `/home/openarm/humanoid_sim_ws/presentation/fixed_base_workspace_demo_short.mp4`, 39.944 s, 1920×1080, 30 fps, H.264/MP4, 5633326 bytes
- All frames decoded: full `True`, short `True`
- Representative frames: `[('/home/openarm/humanoid_sim_ws/presentation/frame_c0.png', (1920, 1080)), ('/home/openarm/humanoid_sim_ws/presentation/frame_c3.png', (1920, 1080)), ('/home/openarm/humanoid_sim_ws/presentation/frame_combined_only.png', (1920, 1080))]`
- Representative frame visual review: PASS — robot, workspace cloud, validated text panel, and combined-only target/valid C3 pose are visible and readable.

## Safety

- trajectory execution = false
- controller = false
- ros2_control = false
- hardware = false
- AMR motion = false
- navigation = false
- Gazebo physics = false

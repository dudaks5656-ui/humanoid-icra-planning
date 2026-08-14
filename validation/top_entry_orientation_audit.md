# Top-entry orientation audit

- TCP link: `openarm_left_hand_tcp`
- TCP joint: fixed child of `openarm_left_link7`, origin XYZ/RPY `[0, 0, 0] / [0, 0, 0]`
- Verified TCP approach axis: local `+Z`
- Finger motion axes at the gripper parent frame: local `-Y` and local `+Y` (mimic pair)
- Required world approach direction: `-Z`
- Required world finger-closing direction: `±Y`

## Selected rotation

- RPY: `[0, pi, 0]` rad
- Quaternion xyzw: `[0, 1, 0, 0]`
- Equivalent quaternion sign: `[0, -1, 0, 0]`
- Rotation matrix:

```text
[-1  0  0]
[ 0  1  0]
[ 0  0 -1]
```

Axis mapping:

- TCP local `+Z -> world -Z` — vertical top entry
- TCP local `+Y -> world +Y` — finger closing surfaces remain aligned with the object's Y faces
- TCP local `+X -> world -X`

## Stage pose derivation

The previously measured `tcp_to_grasp_center=0.13545987646484376 m` lies along TCP local `+Z`. Under the selected orientation this vector points along world `-Z`, so TCP Z is grasp-center Z plus this distance.

| Stage | Grasp-center Z rule | TCP XYZ [m] | Quaternion xyzw |
|---|---|---|---|
| APPROACH | top rim `1.090` + provisional `0.010` | `[0.675, 0.200, 1.235459876465]` | `[0, 1, 0, 0]` |
| PRE_GRASP | object top `0.990` + provisional `0.010` | `[0.675, 0.200, 1.135459876465]` | `[0, 1, 0, 0]` |
| GRASP | object center `0.965` | `[0.675, 0.200, 1.100459876465]` | `[0, 1, 0, 0]` |

The two 10 mm values are `PROVISIONAL_AUDIT_CLEARANCE`, not final task parameters. No object attachment, trajectory, or path planning was performed.


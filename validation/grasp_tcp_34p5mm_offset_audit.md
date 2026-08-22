# Grasp TCP 34.5 mm Offset Audit

Date: 2026-08-22

Robot source: `src/humanoid_description/urdf/humanoid.urdf.xacro`

MoveIt/workspace TCP source of truth: `openarm_left_hand_tcp` (left), `openarm_right_hand_tcp` (right)

The expanded pre-change model had a zero TCP translation for both hands. Therefore the
mathematical expression `v / ||v||` is undefined for that pre-change TCP vector. The applied
direction was not guessed: both existing finger-joint origins are displaced from their respective
`openarm_*_link7` parent along local `+Z` (`0 0 0.1025`), and the existing grasp-geometry audit
independently identifies the grasp-center direction as TCP local `+Z`. Both mirrored hands therefore
use the independently confirmed local unit direction `[0, 0, 1]`.

## LEFT

- frame: `openarm_left_hand_tcp`
- joint: `openarm_left_hand_tcp_joint` (fixed)
- parent: `openarm_left_link7`
- old xyz: `[0, 0, 0]` m
- old offset magnitude: `0.000000` m
- new xyz: `[0, 0, 0.0345]` m
- offset unit vector: `[0, 0, 1]` (existing gripper local `+Z` direction)
- added distance: `0.034500` m
- old orientation: RPY `[0, 0, 0]` rad
- new orientation: RPY `[0, 0, 0]` rad
- orientation delta: `0` rad
- orientation unchanged: **PASS**

## RIGHT

- frame: `openarm_right_hand_tcp`
- joint: `openarm_right_hand_tcp_joint` (fixed)
- parent: `openarm_right_link7`
- old xyz: `[0, 0, 0]` m
- old offset magnitude: `0.000000` m
- new xyz: `[0, 0, 0.0345]` m
- offset unit vector: `[0, 0, 1]` (existing mirrored gripper local `+Z` direction)
- added distance: `0.034500` m
- old orientation: RPY `[0, 0, 0]` rad
- new orientation: RPY `[0, 0, 0]` rad
- orientation delta: `0` rad
- orientation unchanged: **PASS**

## Validation

- xacro parse: **PASS**
- URDF validation (`check_urdf`): **PASS**
- root/parent-child chain retained: **PASS**
- left parent retained (`openarm_left_link7`): **PASS**
- right parent retained (`openarm_right_link7`): **PASS**
- left FK old TCP at zero joint configuration: `[0, 0.422500142484703, 0.474000040032398]` m
- left FK new TCP at zero joint configuration: `[0, 0.422500153759296, 0.439500040032400]` m
- left FK TCP delta: `0.034500000000000` m
- right FK old TCP at zero joint configuration: `[0, -0.422500142484703, 0.474000040032398]` m
- right FK new TCP at zero joint configuration: `[0, -0.422500153759296, 0.439500040032400]` m
- right FK TCP delta: `0.034500000000000` m
- left 34.5 mm check (`abs(delta - 0.0345) <= 1e-6`): **PASS**
- right 34.5 mm check (`abs(delta - 0.0345) <= 1e-6`): **PASS**
- left orientation delta: `0` rad (**PASS**)
- right orientation delta: `0` rad (**PASS**)
- generated URDF differences limited to the two TCP `origin xyz` values: **PASS**
- collision geometry unchanged: **PASS** (24 generated collision elements exact-match)
- visual geometry unchanged: **PASS** (24 generated visual elements exact-match)
- inertial data unchanged: **PASS** (20 generated inertial elements exact-match)
- joint limits unchanged: **PASS** (21 generated limit elements exact-match)
- joint axes unchanged: **PASS** (21 generated axis elements exact-match)
- SRDF/ACM unchanged: **PASS** (`7892b122fe28e91650f8919a66c256ebc55d3b34472e006701a63cadab5484e8`)
- MoveIt joint limits unchanged: **PASS** (`1e851fa791dab1c95eeba4a698e620c0334afd12a0c9964b774f24868d03ccfb`)
- MoveIt kinematics unchanged: **PASS** (`7a825c85e6d3bfa06b3ff6d195b3aced8c490d6132d8d94e2d0b01f52e73204e`)
- OMPL settings unchanged: **PASS** (`3e47c074ffcb9a72ea62e8821a8923f80c7f346e246993b3131038380aaf1335`)
- unrelated model settings unchanged: **PASS**

## Change Scope

- `src/humanoid_description/urdf/openarm_v10_arms_adapter.xacro`: assigns the same verified
  `0 0 0.0345` local TCP translation to both hand instances.
- `src/openarm_description/assets/robot/openarm_v1.0/urdf/ee/ee_dispatcher.xacro`: forwards the
  already-declared `tcp_xyz` and `tcp_rpy` arguments to the existing gripper macro. No frame was
  added and `tcp_rpy` remains `0 0 0`.
- No gripper/finger geometry, collision geometry, joint origin other than the two TCP fixed-joint
  origins, joint limit, SRDF/ACM, kinematics, or OMPL setting was changed.
- No workspace/grid/FK-boundary/differential analysis, presentation figure, or video was recomputed.

## Overall

- LEFT TCP correction: **PASS**
- RIGHT TCP correction: **PASS**
- Ready for a separately authorized workspace recomputation: **YES**

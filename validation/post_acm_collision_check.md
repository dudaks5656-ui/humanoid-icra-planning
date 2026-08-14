# Post-ACM Collision Scope Verification

Date: 2026-08-12 (Asia/Seoul)  
Robot description: user-validated mount coordinates, unchanged numerically  
Collision detector: FCL

## Exact SRDF change

Exactly one new entry was added:

```xml
<disable_collisions link1="waist_pitch_link" link2="openarm_left_link0" reason="Adjacent"/>
```

Reason: the user validated the physical shoulder placement; the two links have no relative motion across the two-fixed-joint mount chain. No finger pair or any other waist/arm pair was added.

SRDF SHA-256:

```text
before: 083268f69bb0aa7ec6120cff141ac11185ee85f2c91c313cea07cc5116f61bdb
after:  7892b122fe28e91650f8919a66c256ebc55d3b34472e006701a63cadab5484e8
```

The ACM contains 23 allowed pairs: the previous 22 adjacent pairs plus this one requested mount pair.

## Build and parser checks

- `humanoid_sim_moveit_config` standalone build: PASS (`1 package finished`)
- Xacro/XML/`check_urdf`: PASS
- SRDF XML parsing and SRDFDOM model initialization: PASS
- planning scene collision environment dynamic type: FCL (`FCL_ACTIVE=1`)
- `SCOPE_OK=1`
- `COLLISION_OK=1`

## Zero-state raw versus ACM-filtered collision

| Pair | ACM allowed | Raw FCL | After ACM | Result |
|---|---:|---:|---:|---|
| `waist_pitch_link ↔ openarm_left_link0` | yes | 346 contacts | 0 | requested pair only |
| `waist_pitch_link ↔ openarm_left_link1` | no | 0 | 0 | still actively checked |
| `waist_pitch_link ↔ openarm_right_link0` | no | 0 | 0 | still actively checked |
| left opposing fingers | no | 110 contacts | 110 contacts | still detected |
| right opposing fingers | no | 110 contacts | 110 contacts | still detected |

Individual ACM inspection confirms `acm_allowed=0` for:

- `waist_pitch_link ↔ openarm_left_link1` through `openarm_left_link7`
- `waist_pitch_link ↔ openarm_right_link0` through `openarm_right_link7`
- both opposing-finger pairs

## Basic and small perturbation states

The following explicit states were checked with raw FCL and the SRDF ACM:

- all independent joints at zero
- Yaw +5° and -5°
- Pitch +5° and -5°
- left joint1 +5°
- right joint1 -5°
- both independent finger joints at 0.011 m

At q=0, both opposing finger collisions remain visible after ACM. At finger q=0.011 m, those finger collisions clear; the newly allowed shoulder mount pair remains the only removed non-default contact.

## Moving waist/arm collision regression

A deterministic generator with seed `20260812` checked 191 full 19-variable states before finding both right- and left-side moving-link collision examples. These are validation samples, not safe-state tables.

Right-side example (`deterministic_random_26`) remained colliding after ACM with:

- `openarm_right_link5 ↔ waist_pitch_link`
- `openarm_right_link5 ↔ waist_yaw_link`
- `openarm_right_link6 ↔ waist_pitch_link`
- `openarm_right_link6 ↔ waist_yaw_link`
- `openarm_right_link7 ↔ waist_yaw_link`

Left-side example (`deterministic_random_190`) remained colliding after ACM with:

- `lift_fixed_link ↔ openarm_left_link4`
- `lift_fixed_link ↔ openarm_left_link5`
- `openarm_left_link7 ↔ waist_pitch_link`
- `openarm_left_right_finger ↔ waist_pitch_link`
- `openarm_left_right_finger ↔ waist_yaw_link`

This demonstrates that collisions involving movable arm links, the opposite arm, grippers, Lift, Yaw, and Pitch remain active except for the single user-authorized fixed mount pair.

Detailed states and contact counts are stored in:

`/home/openarm/humanoid_sim_ws/validation/acm_scope_verification.csv`

## Unchanged gripper policy

No gripper ACM entry, joint limit, joint origin, collision mesh, or mimic definition was changed. The lower-end finger collision remains intentionally visible for later physical open/closed convention review.

## Runtime safety

No RViz, MoveIt `move_group`, controller manager, ros2_control node, hardware interface, Serial/CAN/USB node, or real robot driver was launched during this transition. If planning visualization is subsequently launched, only the existing planning-only launch is permitted; trajectory execution remains disabled.

A final read-only graph query with `ROS_DOMAIN_ID=42` and `ROS_LOCALHOST_ONLY=1` returned no ROS nodes.

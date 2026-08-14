# Top-open box visibility and placement audit

- Scene: `TOP_OPEN_BOX_600X400X150_GEOMETRY_VALIDATION`
- Date: 2026-08-12 (Asia/Seoul)
- Fixed frame: `world`
- Scope: display, AABB, and geometric reach screening only
- MoveGroup, IK, OMPL, trajectory generation/execution, controllers, and hardware: **not used**

## RViz display diagnosis

The box markers were being published correctly, but the original RViz configuration used the property name `Marker Topic`. ROS 2 Humble's MarkerArray display expects `Topic`; consequently the marker publisher initially reported zero subscribers even though the display appeared enabled.

After changing only the RViz display property:

- Publisher: `/top_open_box_geometry_publisher`
- Topic: `/top_open_box_geometry_markers`
- Message: `visualization_msgs/msg/MarkerArray`
- Publisher count: `1`
- RViz subscriber count: `1`
- Reliability: `RELIABLE`
- Durability: `TRANSIENT_LOCAL`
- Lifespan: infinite
- Marker lifetime: `0` (persistent)
- All marker `frame_id` values: `world`
- Fixed Frame: `world`
- All five box scales: non-zero
- RViz runtime state: `RViz is ready`; no MarkerArray display error was logged

Visibility was strengthened without moving geometry:

- Floor: gray, alpha `0.72`
- Front wall: blue, alpha `1.00`
- Back wall: red, alpha `0.80`
- Left wall: green, alpha `0.80`
- Right wall: yellow, alpha `0.80`
- Internal usable-space wireframe: white, alpha `1.00`, line width `0.014 m`
- Large XYZ axes and `BOX CENTER` text at `[0.675, 0.200, 1.015] m`
- Camera focal point: `[0.675, 0.200, 1.015] m`
- Camera distance: `2.2 m`

Conclusion: the original invisibility was a **RViz topic-property/configuration problem**, made harder to recognize by a distant camera. It was not caused by robot occlusion or spatial overlap.

## Robot and current-box AABB result

Signed axis gaps in the CSV use positive values for separation and negative values for overlap along that axis. Full AABB overlap requires overlap on X, Y, and Z simultaneously.

| Category | Geometry | X range [m] | Y range [m] | Z range [m] | Minimum box distance [m] | AABB overlap |
|---|---|---|---|---:|---|
| Fixed body | visual | `[-0.353407, 0.365458]` | `[-0.234781, 0.235239]` | `[0.000001, 1.614873]` | `0.084542` | no |
| Fixed body | collision | `[-0.353407, 0.365458]` | `[-0.234781, 0.235239]` | `[0.000001, 1.614873]` | `0.084542` | no |
| Left arm + gripper | visual | `[0.205976, 0.303976]` | `[0.029229, 0.224227]` | `[0.934871, 1.614873]` | `0.146024` | no |
| Left arm + gripper | collision | `[0.206039, 0.303985]` | `[0.030228, 0.223788]` | `[0.934952, 1.614873]` | `0.146015` | no |
| Right arm + gripper | visual | `[0.205976, 0.303976]` | `[-0.223769, -0.028771]` | `[0.934871, 1.614873]` | `0.146024` | no |
| Right arm + gripper | collision | `[0.206039, 0.303985]` | `[-0.223331, -0.029771]` | `[0.934952, 1.614873]` | `0.146015` | no |
| AMR | visual/collision | `[-0.353407, 0.365458]` | `[-0.234781, 0.235239]` | `[0.000001, 0.375500]` | `0.546084` | no |
| Waist | visual/collision | `[0.194980, 0.314976]` | `[-0.042271, 0.040184]` | `[1.205500, 1.614873]` | `0.177685` | no |
| Whole robot | visual | `[-0.353407, 0.365458]` | `[-0.234781, 0.235239]` | `[0.000001, 1.614873]` | `0.084542` | no |
| Whole robot | collision | `[-0.353407, 0.365458]` | `[-0.234781, 0.235239]` | `[0.000001, 1.614873]` | `0.084542` | no |

Current box:

- Outer AABB: X `[0.450, 0.900]`, Y `[-0.125, 0.525]`, Z `[0.915, 1.090]` m
- Inner AABB: X `[0.475, 0.875]`, Y `[-0.100, 0.500]`, Z `[0.940, 1.090]` m
- Conservative fixed-body front: `robot_fixed_front_x = 0.365457855225 m`
- Current front inner plane offset: `0.475 - 0.365457855225 = 0.109542144775 m` ahead of the fixed body
- Current front wall outer surface offset: `0.450 - 0.365457855225 = 0.084542144775 m`

Neither visual nor collision AABB overlaps the current box. There is also no case where collision geometry is separated but visual geometry overlaps.

## Visualization-only placement candidates

The current five collision objects remain at their audited location. A/B/C are wireframe/text markers only and were not added to the collision world.

| Candidate | Front inner X [m] | Back inner X [m] | Actual outer-wall clearance [m] | Target center XYZ [m] | Fixed-body/AMR/waist overlap |
|---|---:|---:|---:|---|---|
| A | `0.465457855225` | `0.865457855225` | `0.075` | `[0.665457855225, 0.200, 0.980]` | none |
| B | `0.565457855225` | `0.965457855225` | `0.175` | `[0.765457855225, 0.200, 0.980]` | none |
| C | `0.665457855225` | `1.065457855225` | `0.275` | `[0.865457855225, 0.200, 0.980]` | none |

The nominal 100/200/300 mm values measure from the fixed-body front to the **inner** front plane. Because the 25 mm provisional front wall is outside that plane toward the robot, the actual outer-wall clearances are 75/175/275 mm.

## Approximate reach screening without IK

The conservative arm-length upper bound was computed as the sum of the URDF joint-origin translation lengths from each `openarm_*_link0` to its hand TCP: `0.623598893884 m` for both arms. Distances below use the default displayed torso state and the candidate target center.

| Candidate | Left link0 distance [m] | Left within bound | Right link0 distance [m] | Right within bound |
|---|---:|---|---:|---|
| A | `0.725705249354` | no | `0.742038173760` | no |
| B | `0.786603166593` | no | `0.801696378982` | no |
| C | `0.854892375945` | no | `0.868800044214` | no |

Thus none of the three **box-floor-center targets** lies inside the basic-pose arm-length sphere. This is only a necessary geometric bound check—not IK or planning—and does not assess later Lift/Yaw/Pitch repositioning or top-down grasp geometry.

## Direct answers

- Was the MarkerArray published? **Yes.** It was valid and persistent; the original RViz configuration did not subscribe to it.
- Why was the box not visible? **RViz topic-property error plus an overly distant camera**, not spatial overlap.
- Does the current box overlap the robot by visual or collision AABB? **No.**
- How far is X=0.475 m from the fixed body front? **109.542 mm ahead by inner-plane definition; 84.542 mm to the outer front-wall surface.**
- Which candidates avoid fixed body, AMR, and waist overlap? **A, B, and C all avoid overlap.**
- Are their floor-center targets within the default-pose geometric arm reach bound? **No for A, B, and C.**

No candidate is selected or committed by this audit. User choice is required before changing the box collision-object placement or starting any later task-stage work.

# Top-open box geometry audit

- Scene ID: `TOP_OPEN_BOX_600X400X150_GEOMETRY_VALIDATION`
- Audit date: 2026-08-12 (Asia/Seoul)
- Fixed frame: `world`
- Purpose: geometry and initial static-overlap validation only
- Planning/IK/trajectory execution: **not performed**

## Source values and derivation

The new scene is stored separately from `confined_scene.yaml`. The retained source values are:

- Existing box center Y: `0.200 m`
- Existing inner-floor surface: `1.12 - 0.36 / 2 = 0.940 m`
- Existing wall thickness: `0.025 m` (`PROVISIONAL_WALL_THICKNESS`)
- Existing floor thickness: `0.025 m` (`PROVISIONAL_WALL_THICKNESS`)
- Existing target size: `[0.050, 0.050, 0.080] m`
- Retained front inner plane: `world X = 0.475 m`

The original front-open configuration remains unchanged with SHA-256
`500c8af08ca14cb6161263de84d80cddae0885c6f8bb7feec35e4ceb10d2d5c3`.

## Internal usable space

| Quantity | Value |
|---|---:|
| Width Y | `0.600 m` |
| Depth X | `0.400 m` |
| Height Z | `0.150 m` |
| X range | `[0.475, 0.875] m` |
| Y range | `[-0.100, 0.500] m` |
| Z range | `[0.940, 1.090] m` |
| Front inner plane | `X = 0.475 m` |
| Back inner plane | `X = 0.875 m` |
| Right inner plane | `Y = -0.100 m` |
| Left inner plane | `Y = 0.500 m` |
| Floor inner plane | `Z = 0.940 m` |
| Top rim/open plane | `Z = 1.090 m` |

All walls are placed outside these inner planes, so they do not reduce the specified 600 x 400 x 150 mm usable volume.

## Five independent collision objects

| Object | Center XYZ [m] | Size XYZ [m] | Display |
|---|---|---|---|
| `box_floor` | `[0.675, 0.200, 0.9275]` | `[0.450, 0.650, 0.025]` | translucent gray |
| `box_front_wall` | `[0.4625, 0.200, 1.015]` | `[0.025, 0.650, 0.150]` | translucent blue |
| `box_back_wall` | `[0.8875, 0.200, 1.015]` | `[0.025, 0.650, 0.150]` | translucent gray |
| `box_left_wall` | `[0.675, 0.5125, 1.015]` | `[0.400, 0.025, 0.150]` | translucent gray |
| `box_right_wall` | `[0.675, -0.1125, 1.015]` | `[0.400, 0.025, 0.150]` | translucent gray |

No top wall, ceiling, solid enclosing box, or enclosing collision boundary was created. The PlanningScene message was read back from ROS and contained exactly the five object IDs above.

## Display-only target

- Center: `[0.675, 0.200, 0.980] m`
- Size: `[0.050, 0.050, 0.080] m`
- Placement: center of the internal X/Y area, resting on the inner floor plane
- Lifecycle: marker only; not attached, moved, planned to, or inserted into the local collision world in this stage

## Static robot-box collision result

- State: Lift `0`, Yaw `0`, Pitch `0`, left/right independent finger joints `0.044 m`; other active joints at the URDF/SRDF default state
- Collision engine: local MoveIt PlanningScene using the installed default FCL detector
- Five box objects checked against the robot: **no collision detected**
- Result file: `top_open_box_static_collision_check.csv`
- This is a single static initial-state check, not a path, reachability, IK, joint-space, or trajectory validation.

## Runtime scope

Launched processes:

1. `robot_state_publisher`
2. `top_open_box_geometry_publisher`
3. `rviz2`

The `/transform_listener_impl_*` graph entry is RViz's internal TF listener, not a separately launched robot/control node. No MoveGroup, OMPL, IK solver call, controller manager, `ros2_control`, trajectory generator, trajectory executor, Serial/CAN/USB node, or hardware node was started.

Runtime warnings were limited to an unused kinematics-plugin warning in the local non-IK model loader and existing RViz inertia-visualization warnings for the four finger links. Neither warning changed the box geometry or the static robot-box collision result.

## Separation and protection

- Front-open development result archive: `validation/front_open_development_reference_20260812`
- New top-open geometry config: `src/humanoid_extraction_experiments/config/top_open_box_600x400x150.yaml`
- No prior reference CSV/YAML/report was edited or recalculated.
- The robot Xacro, SRDF/ACM, joint limits, collision meshes, kinematics, and OMPL configuration retain their pre-task SHA-256 values.
- No new reference trajectory was generated.

## User visual approval pending

The active RViz session must be checked for: front wall on the robot-facing side, top-only opening, natural 12:8:3 proportions, scale relative to the robot, absence of initial AMR/robot overlap, target placement at the center of the floor, and visibly separate five faces. No approach, descent, grasp, lift, transfer, reference generation, XYZ expansion, torso recovery, or timing experiment may begin before approval.

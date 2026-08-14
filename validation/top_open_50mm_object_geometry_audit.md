# 50 mm Pick-object geometry audit

- Date: 2026-08-12 (Asia/Seoul)
- Scene: `TOP_OPEN_BOX_600X400X150_GEOMETRY_VALIDATION`
- Object: user-confirmed `50 x 50 x 50 mm` cube
- Scope: geometry, marker, and static collision definition only

## Main box preservation

The main box was not moved or resized:

- Internal X: `[0.475, 0.875] m`
- Internal Y: `[-0.100, 0.500] m`
- Internal Z: `[0.940, 1.090] m`
- Internal size: `0.400 x 0.600 x 0.150 m` in X/Y/Z
- Wall and floor thickness: `0.025 m` (`PROVISIONAL_WALL_THICKNESS`)
- Collision objects: floor, front, back, left, and right; top remains open

The pre-change YAML is preserved at
`validation/backups/top_open_box_600x400x150_before_50mm_20260812.yaml`.

## Target placement

- Size XYZ: `[0.050, 0.050, 0.050] m`
- Center XYZ: `[0.675, 0.200, 0.965] m`
- Target AABB X: `[0.650, 0.700] m`
- Target AABB Y: `[0.175, 0.225] m`
- Target AABB Z: `[0.940, 0.990] m`

Vertical result:

- Target bottom: `0.965 - 0.025 = 0.940 m`
- Floor inner surface: `0.940 m`
- Gap: `0 m`
- Penetration: `0 m`
- Target top: `0.965 + 0.025 = 0.990 m`

Horizontal clearance to inner wall planes:

- Front: `0.650 - 0.475 = 0.175 m`
- Back: `0.875 - 0.700 = 0.175 m`
- Right: `0.175 - (-0.100) = 0.275 m`
- Left: `0.500 - 0.225 = 0.275 m`

The target touches only the intended floor plane and does not intersect any vertical wall.

## Gripper nominal aperture

- User-confirmed internal gap at `q_open=0.044 m`: `72.48 mm`
- Object width along closing direction Y: `50.00 mm`
- Total nominal free width: `22.48 mm`
- Nominal per-side clearance: `(72.48 - 50.00) / 2 = 11.24 mm`

This lateral clearance does not guarantee bottom clearance during a vertical grasp; that question is evaluated by the separate IK/FCL audit.

## RViz state

The target marker reads the same YAML size and explicit center used by the reachability audit. A/B/C candidate markers were removed; only the current fixed box, target, internal wireframe, axes, and text are published.


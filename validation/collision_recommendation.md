# Collision Recommendation — Audit Only

No Xacro, SRDF, ACM, joint limit, collision mesh, controller, or hardware configuration change is authorized by this report.

## 1. Coordinate error assessment

**Finding: the active model still uses explicit provisional symmetric mount coordinates.**

The Inventor YAML coordinates are correctly derived from `T_pitch^-1 * T_mount` in `ucs_matrices.json`. The active Xacro instead loads `provisional_left_mount_xyz` and `provisional_right_mount_xyz`. A common frame rotation, translation, or the subsequent ±90° adapter rotations cannot explain the two different coordinate pairs.

Recommendation: before editing the active Xacro, confirm that the delivered Inventor UCS package corresponds to the physically validated assembly revision and that the mount UCS is intended to coincide with each OpenArm link0 adapter origin. If confirmed, propose a separately reviewed source change with backup and before/after hashes.

## 2. Collision-mesh assessment

**Finding: no scale-magnitude or mesh-size defect was found for link0.**

The left visual and collision meshes both use the OpenArm macro's deliberate negative-Y reflection; the right uses positive-Y. Metric AABB extents and absolute mesh volume match. Reflection reverses winding/sign, but FCL loads the mesh and the left contact disappears in the Inventor-position comparison without any mesh change.

Recommendation: do not alter or re-export the mesh based on this audit. If contact-normal direction becomes important for minimum-distance analysis, perform a dedicated FCL/Bullet comparison later, without changing the source mesh first.

## 3. SRDF allowed-collision assessment

### `waist_pitch_link ↔ openarm_left_link0`

**Not eligible for ACM disabling.** The current penetration is approximately 38 mm and 346 contacts. It becomes zero when only the mount XYZ is replaced by the delivered Inventor values. This is evidence for mount-coordinate resolution, not an intended flange contact.

### Opposing finger pairs

**Not yet eligible for ACM disabling.** Both sides collide only in the sampled lower-end interval q=0.0000–0.0066 m and are clear from q=0.0088–0.0440 m. There is no authoritative named state proving the physical open/closed meaning of the range endpoints. The next step is to verify the real gripper zero convention and visually inspect grasp surfaces before considering a joint-limit correction, origin correction, or normal closure contact.

## Hold point

- Do not add any of the three pairs to `disable_collisions`.
- Do not change the current Xacro mount coordinates yet.
- Do not change finger limits or origins yet.
- If RViz/MoveIt is relaunched after approval, keep it planning-only and hardware-free.
- Await user/CAD confirmation of the mount UCS revision and physical gripper endpoint convention.

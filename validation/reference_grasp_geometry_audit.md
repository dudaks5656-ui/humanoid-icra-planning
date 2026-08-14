# REFERENCE_GRASP geometry audit for the 50 mm cube

## Scope and reused evidence

- Selection scope: `CENTRAL_OBJECT_ONLY`
- Reuse policy: `FIXED_FOR_ALL_XYZ_AND_ALL_ABLATIONS`
- Object: 50 mm cube at `[0.675, 0.200, 0.965]` m, resting on floor Z=0.940 m
- Orientation: RPY `[0, pi, 0]`, quaternion xyzw `[0, 1, 0, 0]`
- Approach: TCP local +Z to world -Z
- Closing direction: object world +/-Y faces
- No +/-X grasp, roll search, OMPL, trajectory, attachment, controller, or hardware was used.

The aperture, collision AABBs, orientation, and explicit-seed IK results from the immediately preceding 50 mm audit were reused. They were not recomputed from the beginning.

## Aperture

- `q_open = 0.044 m`: collision-AABB inside gap 0.0724776229858398 m.
- `q_contact_50mm = 0.0327611885070801 m`: inside gap 0.0500000000000000 m.
- Object penetration at the selected aperture: 0 m within numerical precision.
- The opposing inner faces are at TCP-frame Y=-0.025 m and +0.025 m, so contact is symmetric about the target centerline.
- Classification: `KINEMATIC_CONTACT_CONFIGURATION_FOR_50MM_OBJECT`.
- This is not a force-grasp validation.

## Finger geometry in the top-entry orientation

- Common inner grasp-face TCP-local Z range: `[0.0879994882812499, 0.182920264648437]` m.
- Collision geometry vertical height: 0.0949207763671876 m.
- TCP-to-finger-bottom downward offset: 0.182920264648437 m.
- TCP-to-inner-face vertical center downward offset: 0.135459876464844 m.
- Zero-clearance minimum grasp-center height above the object bottom: 0.0474603881835938 m.
- Minimum vertical overlap criterion: 0.015 m, development-only and not a physical grasp-stability result.

## Approved height candidates

| Height above bottom | Finger-floor clearance | Vertical overlap | Symmetric side contact | Result |
|---:|---:|---:|---|---|
|0.030 m|-0.0174603882 m|0.050 m|yes|floor collision|
|0.035 m|-0.0124603882 m|0.050 m|yes|floor collision|
|0.040 m|-0.0074603882 m|0.050 m|yes|floor collision|
|0.045 m|-0.0024603882 m|0.050 m|yes|floor collision|

At all four heights, both q_open and q_contact retain the same vertical finger extent because the prismatic finger joints move only along local +/-Y. The only observed rejection pair at the audited central pose was each finger against `box_floor`; no vertical wall, non-finger/target, self-collision, or joint-limit failure caused this gate failure.

## Selection result

`REFERENCE_GRASP` was **not selected**. All four explicitly approved heights penetrate the floor. The nearest candidate, 45 mm, still penetrates by 2.460388 mm. A previously computed mathematical zero-clearance height of 47.460388 mm was deliberately not promoted because it is outside the newly fixed candidate set.

Per the stop condition, neither the robot/object/box geometry nor the candidate list was altered, and common-grasp reachability/OMPL reference generation was not started.

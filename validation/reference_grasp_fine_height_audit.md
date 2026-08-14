# 50 mm grasp-height geometry audit

## Fixed inputs and frame mapping

- Target: 0.050 m cube, center [0.675, 0.200, 0.965] m, Z [0.940, 0.990] m
- Top-entry orientation: RPY [0, pi, 0], quaternion xyzw [0, 1, 0, 0]
- TCP local +Z -> world -Z; local +/-Y -> world +/-Y
- Global SRDF/ACM and robot collision geometry were not changed

## Finger collision AABBs in TCP frame

|State/link|X [m]|Y [m]|Z [m]|
|---|---|---|---|
|q_open left|[-0.0304651, 0.030501]|[0.0362388, 0.0710614]|[0.0879995, 0.18292]|
|q_open right|[-0.0304651, 0.030501]|[-0.0710614, -0.0362388]|[0.0879995, 0.18292]|
|q_contact left|[-0.0304651, 0.030501]|[0.025, 0.0598226]|[0.0879995, 0.18292]|
|q_contact right|[-0.0304651, 0.030501]|[-0.0598226, -0.025]|[0.0879995, 0.18292]|

- q_open: 0.044 m; inside gap=0.0724776 m
- q_contact_50mm: 0.0327611885070801 m; inside gap=0.05 m
- Classification: `KINEMATIC_CONTACT_CONFIGURATION_FOR_50MM_OBJECT`; this is not force-grasp validation
- Actual inward-facing grasp triangles, left local-Z range: [0.170295997070313, 0.182712806152344] m
- Actual inward-facing grasp triangles, right local-Z range: [0.170295997070313, 0.182712806152344] m
- Common actual inner-face local-Z range: [0.170295997070313, 0.182712806152344] m
- Actual inner-face extraction: triangles parallel to the inward Y plane at the collision-mesh inner boundary (|normal.Y| >= 0.90).
- Selected mesh triangles: left=4, right=4
- Full finger collision-AABB vertical height: 0.0949207763671876 m
- TCP-to-finger-bottom downward offset under top entry: 0.182920264648437 m
- TCP-to-full-collision-AABB center downward offset used by the existing grasp-height convention: 0.135459876464844 m
- TCP-to-actual-inner-grasp-face center downward offset: 0.176504401611328 m
- The fine search preserves the existing AABB-center grasp-height convention; the actual inner face is used only for overlap measurement.

## Floor-clearance-derived minimum grasp-center heights

|Required floor clearance [m]|Minimum grasp-center height above object bottom [m]|
|---|---|
|0|0.0474603881835938|
|0.001|0.0484603881835938|
|0.002|0.0494603881835938|
|0.003|0.0504603881835938|
|0.005|0.0524603881835938|

The 15 mm vertical overlap threshold is a development-only minimum contact-height criterion, not a final physical grasp-stability criterion.

## Selection

No candidate simultaneously met floor clearance, >=15 mm overlap, symmetric aperture, and collision-free IK. Robot/object geometry was not changed.

| Height | Floor clearance | Actual inner-face/object overlap | q_open free IK | q_contact free IK | Result |
|---:|---:|---:|---:|---:|---|
|47.5 mm|0.039612 mm|12.416809 mm|30/30|30/30|overlap below 15 mm|
|48.0 mm|0.539612 mm|12.416809 mm|30/30|30/30|overlap below 15 mm|
|48.5 mm|1.039612 mm|12.416809 mm|30/30|30/30|overlap below 15 mm|
|49.0 mm|1.539612 mm|12.416809 mm|30/30|30/30|overlap below 15 mm|
|49.5 mm|2.039612 mm|12.416809 mm|30/30|30/30|overlap below 15 mm|
|50.0 mm|2.539612 mm|12.416809 mm|30/30|30/30|overlap below 15 mm|

All six candidates clear the floor, satisfy joint bounds, preserve the exact symmetric 50 mm aperture, and produced collision-free IK at the audited central Lift=0.35 m state. The blocker is exclusively the development overlap threshold: the actual inward-facing planar mesh patch itself is only 12.416809 mm tall, so it cannot provide 15 mm overlap at any height.

## Stop condition and alternatives

No `REFERENCE_GRASP` was selected and the full Lift/Yaw/Pitch reachability sweep was not started. The following are proposals only; none was applied:

- use an object with a real upper grasp feature, such as an enlarged bolt head;
- raise the object with a measured 3–5 mm support structure;
- use shorter fingers or improve the finger collision geometry after physical/CAD validation;
- use a different real grasp method.

## Finger state lifecycle (definition only)

- APPROACH/PRE_GRASP/DESCENT entry: q_open
- GRASP closure: q_open -> q_contact_50mm
- LIFT (future work): q_contact_50mm with attached object
- No force control, attachment, trajectory, OMPL, controller, or hardware was used here.

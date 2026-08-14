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
- Common inner grasp-face local-Z range: [0.0879994882812499, 0.182920264648437] m
- Finger total/common vertical height: 0.0949207763671876 m
- TCP-to-finger-bottom downward offset under top entry: 0.182920264648437 m
- TCP-to-inner-grasp-face center downward offset: 0.135459876464844 m

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

- Selected grasp-center height above object bottom: 0.0474603881835938 m
- Finger-floor clearance: 1.11022302462516e-16 m
- Finger/object vertical overlap: 0.0499999999999999 m
- q_open static-pose collision-free IK count: 20 / 30
- q_contact task-ACM collision-free IK count: 30 / 30
- A grasp center above the object center is an intentional side-grasp offset for an object resting on a floor.

## Finger state lifecycle (definition only)

- APPROACH/PRE_GRASP/DESCENT entry: q_open
- GRASP closure: q_open -> q_contact_50mm
- LIFT (future work): q_contact_50mm with attached object
- No force control, attachment, trajectory, OMPL, controller, or hardware was used here.

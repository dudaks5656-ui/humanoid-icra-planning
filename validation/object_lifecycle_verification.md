# Object lifecycle verification

Date: 2026-08-12 (Asia/Seoul)

## Implemented stage policy

| Stage | Object state | Allowed contact | Collision checks retained |
|---|---|---|---|
| APPROACH | `target_object` in world | None | Robot self, robot-box, every robot-target pair |
| INSERTION | `target_object` in world | None | Robot self, robot-box, every robot-target pair |
| GRASP_POSE | `target_object` in world | Task-scoped ACM only for the two left finger links vs target | Non-finger arm-target, robot-box, robot self |
| LIFT | Attached to `openarm_left_hand_tcp` | `touch_links`: two left finger links | Attached-object vs all box walls/floor, robot-box, robot self |
| EXTRACTION | Remains attached | Same two touch links | Attached-object vs all box walls/floor, robot-box, robot self |

The SRDF/global ACM is unchanged. After a successful GRASP trajectory, adding an `AttachedCollisionObject` with the same ID performs the world removal and attachment in one PlanningScene diff. The attached pose is computed from the actual final RobotState, not assumed from the requested pose. Each later candidate resets to the original world-object scene.

## Error categories

The experiment records these distinct reasons: `ROBOT_SELF_COLLISION`, `ROBOT_BOX_COLLISION`, `FINGER_TARGET_PREMATURE_COLLISION`, `NON_FINGER_TARGET_COLLISION`, `ATTACHED_OBJECT_BOX_COLLISION`, `IK_FAILURE`, and `MOTION_PLANNING_FAILURE`.

## Three-run result

All runs used target `[0.50, 0.30, 1.25] m`, target size `[0.05, 0.05, 0.08] m`, the approved orientation, q_open `0.044 m`, the same candidate list, and the same collision policy.

| Run | TORSO_LOCKED first failure | TORSO_CANDIDATE_SEARCH | Furthest repeatable candidate result |
|---|---|---|---|
| 1 | EXTRACTION, `IK_FAILURE` | Failure | Yaw 0 deg and Yaw -10 deg candidates reached EXTRACTION then `IK_FAILURE` |
| 2 | GRASP_POSE, `MOTION_PLANNING_FAILURE` | Failure | Yaw 0 deg and Yaw -10/Pitch -10 deg candidates reached EXTRACTION then `IK_FAILURE` |
| 3 | APPROACH, `MOTION_PLANNING_FAILURE` | Failure | Yaw 0 deg and Yaw -10 deg candidates reached EXTRACTION then `IK_FAILURE` |

Each candidate search evaluated 25 in-limit candidates; five exact 45 deg Pitch values were excluded because they exceed the URDF's rounded upper limit and were not clamped. No candidate completed all five stages.

The external Humble `move_group`/`MoveGroupInterface` path does not expose a supported per-request OMPL RNG seed in the installed API. No unsupported parameter was invented. Therefore the three process repeats are not falsely labelled fixed-seed runs. Their variable early OMPL failures are recorded, while the furthest candidates consistently terminate at the deterministic EXTRACTION IK check before an OMPL request for that stage.

## Why a target-position change would be required next

With the current values, the EXTRACTION TCP target is:

```text
x = 0.50 - 0.135459876 - 0.24 = 0.124540124 m
y = 0.30 m
z = 1.25 + 0.08 = 1.33 m
```

The furthest candidates have no IK at this pose. Moving the object deeper into the box in world +X would move the final extracted TCP target in +X by the same amount while retaining the approved orientation and lifecycle. Per the requested stop condition, no target-position change was made in this step.

No trajectory was executed. No controller, ros2_control node, hardware interface, or real robot node was started. RViz was not opened because no successful recovery trajectory exists.

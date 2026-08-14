# Stage-constrained reference v2: cloud design and local validation

## Status and scope

**LOCAL_VALIDATION_REQUIRED.** This cloud checkout does not establish that ROS 2 Humble, MoveIt, FCL, RViz, or the excluded meshes are complete. No planning run, trajectory execution, controller, `ros2_control`, driver, or hardware command was performed here.

The fixed stage contract is: OMPL `TOP_APPROACH`; fixed-XY/fixed-orientation Cartesian `VERTICAL_DESCENT` with sequential IK; finger-only `GRASP`; attached-object Cartesian `LIFT_CLEAR`; OMPL `TRANSFER_OUTSIDE`.

## Root-cause separation

### Implementation causes

1. The forward prototype ranked many top-pose IK solutions but executed only branch ranks 0 and 1. Its previous-state-only IK continuation therefore equated failure of two local branches with failure of the constrained segment.
2. The reverse prototype likewise executed only ranks 0 and 1 although Lift 0.35 exposed 24 collision-free grasp branches and Lift 0.40 exposed 18–21. It stopped immediately when one previous-state-seeded KDL call failed; it did not try a deterministic alternate IK seed at that same Cartesian waypoint.
3. v1 selected the first full success. It did not enforce the handoff requirement for five dense-valid candidates before selection.
4. Historical launch files write fixed paths in `validation/`, so rerunning them can overwrite preserved evidence. The v2 launch requires a new, nonexistent `output_dir` and refuses reuse through `os.makedirs(..., exist_ok=False)`.

### Structural causes (not changed)

1. The fixed vertical TCP line crosses a narrow/disconnected numerical IK region: forward Lift 0.30 stops at 29/34 intervals, while reverse Lift 0.35 stops at 11/12 and leaves about 14 mm. Smaller spacing alone did not reconnect the branch.
2. Lift 0.20/0.25/0.30 did not provide collision-free non-boundary grasp branches in the reverse search, while Lift 0.35/0.40 did. This is a constrained kinematics/collision property, not evidence that limits or collision models should be relaxed.
3. The two global/random paths used joint-space OMPL for descent/lift, had zero active-joint margin and sub-millimetre clearance, and therefore do not prove existence of a robust fixed-line Cartesian reference.
4. The gripper-open coordinate is at its bound, so all-variable joint margin can be zero even when active arm revolute margin is positive. v2 retains the exact bounds and reports active-revolute margin separately.

## Minimal implementation change

The v1 sources and their outputs remain unchanged. The separate v2 executable:

- deterministically enumerates all 24 ranked branch slots at Lift 0.35 and 0.40;
- preserves Yaw=Pitch=0 and the exact Cartesian poses;
- first uses previous-state sequential IK, then, only at a failed waypoint, tries 32 deterministically seeded IK calls and chooses the collision-free solution nearest to the preceding arm state;
- does not alter or relax Xacro/URDF, SRDF/global ACM, joint limits, collision geometry, kinematics, OMPL, the fixed grasp, or box geometry;
- continues to use task-local finger/object contact only for closure and attaches the target before lift;
- requires at least five complete dense-valid candidates before lexicographic selection.

The multistart fallback samples legal seeds inside exact model bounds; returned states still undergo exact bounds and FCL checks. A longer partial segment is never promoted.

## Exact Ubuntu 22.04 / ROS 2 Humble commands

From the workspace root after restoring the locally preserved licensed meshes:

```bash
source /opt/ros/humble/setup.bash
python3 src/humanoid_extraction_experiments/scripts/check_reference_v1_regression.py \
  --validation-dir validation \
  --output validation/stage_constrained_v2_$(date -u +%Y%m%dT%H%M%SZ)_regression/v1_regression_audit.md

colcon build --symlink-install --packages-select \
  openarm_description humanoid_sim_description humanoid_sim_moveit_config \
  humanoid_extraction_experiments \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash

RUN_ID="stage_constrained_v2_$(date -u +%Y%m%dT%H%M%SZ)"
OUT="$PWD/validation/$RUN_ID"
ros2 launch humanoid_extraction_experiments stage_constrained_reference_v2.launch.py \
  output_dir:="$OUT" hold_for_rviz:=false
```

Do not launch a controller, `ros2_control`, a driver, or a trajectory execution capability. The launch explicitly disables execution. `output_dir` must not already exist.

After the process exits, perform planning-output checks:

```bash
test -s "$OUT/stage_v2_reference_trials.csv"
test -s "$OUT/stage_v2_reference_trajectory.csv"
test -s "$OUT/stage_v2_reference_waypoints.yaml"
test -s "$OUT/stage_v2_reference_audit.md"
python3 - "$OUT" <<'PY'
import csv, pathlib, sys, yaml
out = pathlib.Path(sys.argv[1])
trials = list(csv.DictReader((out / "stage_v2_reference_trials.csv").open()))
assert sum(row["success"] == "1" for row in trials) >= 5
trajectory = list(csv.DictReader((out / "stage_v2_reference_trajectory.csv").open()))
assert trajectory and "status" not in trajectory[0]
assert {row["stage"] for row in trajectory} == {
    "TOP_APPROACH", "VERTICAL_DESCENT", "GRASP", "LIFT_CLEAR", "TRANSFER_OUTSIDE"
}
assert all(row["validity"] == "VALID" for row in trajectory)
manifest = yaml.safe_load((out / "stage_v2_reference_waypoints.yaml").read_text())
assert manifest["status"] == "SELECTED"
assert manifest["planning_only"] is True
assert manifest["trajectory_execution_performed"] is False
PY
sha256sum \
  src/humanoid_sim_description/urdf/humanoid_sim.urdf.xacro \
  src/humanoid_sim_description/urdf/openarm_arms_adapter.xacro \
  src/humanoid_sim_description/config/cad_frame_transforms.yaml \
  src/humanoid_sim_description/config/validated_arm_mount_alignment.yaml \
  src/humanoid_sim_moveit_config/config/humanoid_sim.srdf \
  src/humanoid_sim_moveit_config/config/joint_limits.yaml \
  src/humanoid_extraction_experiments/config/top_open_reference_scene.yaml \
  validation/reference_grasp_50mm.yaml | tee "$OUT/protected_inputs_sha256.txt"
git rev-parse HEAD | tee "$OUT/input_commit.txt"
```

A shutdown-only `move_group` SIGSEGV after the generator has flushed its files must be recorded as a warning in the audit; it must not silently convert missing or malformed artifacts into a pass.

## Success and failure decision

**PASS** requires all of the following:

- at least five trials with `success=1`;
- exactly the five required stages, in order, with continuous stage endpoints;
- dense interpolation no coarser than 0.01 rad revolute / 0.005 m prismatic;
- zero exact joint-limit, self-collision, robot-box, premature target-contact, and attached-object-box violations;
- Yaw and Pitch within 1e-6 rad of zero throughout;
- fixed TCP XY/orientation during descent and lift, endpoints within configured tolerances;
- finger motion only during `GRASP`, target attachment after closure and before `LIFT_CLEAR`;
- selected CSV is a real trajectory, not a two-line `NOT_SELECTED` status artifact.

Anything else is **FAIL**, classified by first failure stage/waypoint/reason. Zero active-revolute margin, any missing raw trajectory, fewer than five candidates, or a low-clearance candidate is not an approvable stable reference. Cloud status remains **LOCAL_VALIDATION_REQUIRED** until the local artifacts are returned.

## Expected curated artifacts

Under the unique `$OUT` directory:

- `stage_v2_reference_trials.csv`: one row per branch attempt, including seed/rank, first failure, fraction, exact margins/clearances/errors, and planning time.
- `stage_v2_reference_trajectory.csv`: every selected state with stage/method/index, all whole-body variables, TCP pose, torso values, attachment state, margins/clearances, and validity.
- `stage_v2_reference_waypoints.yaml`: selected status, planning-only flags, grasp identity, selected trial, stage methods/counts/endpoints, and attached transform.
- `stage_v2_reference_audit.md`: protocol, every attempt, selection/classification, minimum metrics, and explicit no-execution statement.
- `protected_inputs_sha256.txt` and `input_commit.txt`: immutable-input and source revision record, created by the commands above.
- `v1_regression_audit.md`: preserved v1 fixture result, normally in its separate regression run directory.

If v2 cannot obtain five candidates, its trajectory CSV and waypoint YAML must remain explicit `NOT_SELECTED` status artifacts and the audit must say `NO_APPROVABLE_STAGE_V2_REFERENCE`.

## Returning local evidence to cloud

1. Keep ROS logs, build/install/log trees, excluded meshes, screenshots with host details, and large raw files local.
2. Review the unique run directory for host paths or credentials. Add only the small CSV/YAML/audit/hash/commit artifacts listed above.
3. In the audit, record Ubuntu/ROS/MoveIt versions, the exact command, generator exit code, any shutdown-only fault, planning-only/no-hardware status, and where omitted raw data remains.
4. Commit the curated run directory on a new local branch and push it to the private remote. Provide the branch/commit SHA to Cloud Codex; do not paste only a verbal “success”.

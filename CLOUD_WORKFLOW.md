# Cloud and local workflow

## Responsibility split

### Cloud Codex

- Develop and review planner, analysis, test, and documentation code.
- Inspect committed small CSV/YAML/audit artifacts.
- Keep robot limits, stage semantics, grasp lifecycle, and protected-file constraints consistent with `ICRA_PROJECT_HANDOFF.md`.
- Do not claim that cloud-only static checks validate ROS runtime behavior.

### Ubuntu local workstation

- Build ROS 2 Humble packages with colcon.
- Run MoveIt planning-only launches.
- Inspect RViz when explicitly required.
- Run IK, collision, reference-planning, recovery, and repeat experiments.
- Retain excluded meshes, generated workspaces, large raw results, and host logs locally.

## Change exchange

1. Develop and document changes in the private GitHub repository using Cloud Codex.
2. Pull cloud changes onto the Ubuntu local workstation.
3. Restore/use locally preserved mesh assets without committing them.
4. Build and validate locally. Keep trajectory execution disabled; do not start hardware, controllers, or `ros2_control` as part of this research workflow.
5. Save validation results under a new run identifier so existing results are never overwritten.
6. Commit the curated small CSV summaries, YAML status/manifest files, and audit Markdown documents needed to explain the result.
7. Keep raw multi-megabyte IK/repeat CSVs, ROS logs, build/install/log trees, backups, screenshots with host information, and unlicensed meshes local.
8. Push the reviewed validation commit so Cloud Codex can continue from evidence rather than an unverified runtime assumption.

## Required local validation record

Each result commit should identify:

- input commit SHA and protected-input hashes;
- ROS 2/MoveIt environment;
- command or launch entry point;
- planning-only and no-hardware status;
- success/failure classification;
- result/audit paths;
- whether raw data remains local;
- any known shutdown-only `move_group` fault.


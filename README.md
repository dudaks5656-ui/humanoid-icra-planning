# ICRA wheeled humanoid planning research

This private handoff repository contains planning and analysis code for confined-space object extraction by a wheeled humanoid composed of a fixed AMR base, Lift, Waist Yaw/Pitch, and dual OpenArm manipulators.

Read [`ICRA_PROJECT_HANDOFF.md`](ICRA_PROJECT_HANDOFF.md) before changing code or interpreting any result. It records the verified robot model, joint limits, coordinate frames, scene and grasp parameters, preliminary experiments, protected files, known failures, and the next planner task.

## Scope

- ROS 2 Humble and MoveIt planning stack.
- Xacro/URDF/SRDF robot description and planning configuration.
- Planning-only simulation and offline validation tools.
- Stage-constrained reference planning and future stage-aware Lift/Yaw/Pitch local recovery research.

This is **not** a hardware execution repository. Trajectory execution, controllers, `ros2_control`, hardware interfaces, AMR/OpenArm drivers, and physical robot commands are outside the repository's validated scope. Existing launch files are intended to keep MoveIt trajectory execution disabled.

## Cloud limitation

CAD-derived and OpenArm mesh files are intentionally excluded from Git until redistribution rights are confirmed. A cloud checkout can support code development, static review, analysis, and documentation, but may not build a complete robot description or run MoveIt/RViz without the locally preserved mesh assets.

Use [`CLOUD_WORKFLOW.md`](CLOUD_WORKFLOW.md) for the cloud/local exchange process and [`LICENSE_STATUS.md`](LICENSE_STATUS.md) before adding any third-party or CAD-derived asset.

## Fixed-base workspace analysis

The planning-only fixed-base workspace study compares the same 1,440 sampled TCP
positions in the targeted forward region. These are simulation results, not a
measured hardware workspace.

| Configuration | Reachable | Targeted volume | Increase vs. C0 |
|---|---:|---:|---:|
| C0 Arm + Lift | 833 / 1440 | 0.097791 m³ | baseline |
| C1 + Waist Yaw | 1030 / 1440 | 0.120918 m³ | +23.65% |
| C2 + Waist Pitch | 976 / 1440 | 0.114578 m³ | +17.17% |
| C3 + Waist Yaw + Pitch | 1119 / 1440 | 0.131366 m³ | +34.33% |

The 4-way ablation found 65 combined-torso-only points (0.007631 m³) that require
Yaw and Pitch together. Implementation and validated outputs are in
[`src/fixed_base_workspace_analysis`](src/fixed_base_workspace_analysis) and
[`validation`](validation). The CSV-driven RViz presentation demo, screenshots,
and full/short MP4 recordings are documented in
[`presentation/README.md`](presentation/README.md).

Raw CSV, audit, integrity, and manifest files are versioned. Generated
`validation/**/*.png` plots remain excluded by repository policy and can be
recreated with the checked-in post-processing scripts.

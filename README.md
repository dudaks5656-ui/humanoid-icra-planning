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


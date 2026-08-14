# Grasp-seeded reference process-exit audit

- Generator result: `NO_APPROVABLE_GRASP_SEEDED_REFERENCE`
- Generator exit code: `2` (the program's explicit no-reference result)
- Trial CSV rows: `10`; CSV parsing and expected row-count check passed.
- Waypoint status: `NOT_SELECTED`; YAML parsing passed.
- Result files were written and closed before launch shutdown began.
- `robot_state_publisher` exited cleanly after SIGINT.
- `move_group` emitted the known class-loader unload warning and exited `-11` during shutdown, after the generator had finished writing its result files.
- The shutdown fault did not change the planning result classification.
- No RViz process was started because no reference was selected.
- No trajectory execution, controller, ros2_control, or hardware process was used.

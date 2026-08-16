# Dataset audit

- Evaluated case rows: 4320.
- Boundary tasks: Phase 1 128/128; Phase 2 64/64.
- Duplicate unique keys: 0.
- CSV field-width errors: 0; final newline present: True.
- `nan` cells: 22221. They are retained when no IK/posture was selected or a physical-envelope rejection makes joint/path metrics undefined.
- `inf` cells: 0. They are retained only when a collision-distance or bound calculation legitimately reports an unbounded sentinel.
- Protected files changed: 0.
- YAW_PITCH dominance comparisons: 931; violations: 0.
- Postprocessing read the stored CSV only; no planning case was re-executed.
- Workspace areas are ray-polygon sample estimates, not exact continuous workspace areas.
- Deterministic seed-bank repeats measure IK-search sensitivity, not physical repeatability.
- All evidence is planning-only simulation evidence and does not establish hardware performance.

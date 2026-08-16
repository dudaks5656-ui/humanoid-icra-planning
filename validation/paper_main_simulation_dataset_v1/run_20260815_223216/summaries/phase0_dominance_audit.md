# Phase 0 dominance audit

- Existing targets audited: 30.
- Existing YAW_PITCH dominance violations: 19/30.
- Cause: combined refinement did not necessarily include single-axis refined optima; posture-order-derived seed keys differed; scoring prioritized whole active margin rather than Arm 1–7 margin.
- Revalidation: every injected YAW_ONLY/PITCH_ONLY candidate already completed the identical 0.17 m Lift-only extraction with the same model, collision scene and fixed torso/Arm posture. Each is therefore a valid member of the relaxed YAW_PITCH feasible set.
- Correction: paper runner explicitly unions YAW_ONLY, PITCH_ONLY and native YAW_PITCH optima and applies extraction-success, Arm-margin, Joint3/5-margin, environment-clearance, self-clearance, posture-magnitude lexicographic selection.
- Corrected dominance violations: 0/30.
- Status: PASS; Phase 1 may start. Existing axis results were not changed.

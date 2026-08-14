# Pilot experiment conclusion

Status: **PILOT_EXPERIMENT**. These results are method-development evidence and are not final paper performance results.

- Of 30 geometrically feasible targets, LIFT_ONLY completed 27/30 and LIFT_YAW_PITCH completed 30/30 in the boundary search.
- Collision-free IK existed for the three targets that differed in that pilot boundary-search run.
- The subsequent 360-repeat experiment classified all three targets as `NO_MEANINGFUL_TORSO_ADVANTAGE` under the recorded operational thresholds.
- The previously successful torso pose for `target_1` and `target_2` was Yaw = 0 and Pitch = 0, so those cases did not contain a functional Yaw–Pitch difference.
- The non-zero Yaw used for `target_3` did not produce a success-rate advantage in the repeated experiment.
- These pilot results motivated a transition to offline reference-trajectory-guided planning. They must not be reported as final performance results for the paper.

Original files remain in `/home/openarm/humanoid_sim_ws/validation`; this directory contains immutable preservation copies. Integrity is recorded in `pilot_manifest_sha256.txt`.

# Table 2. Same-target single-axis ablation

LOCKED failure rows contain no stored Arm posture; yaw/pitch zero are experimental fixed conditions, not invented IK results. These pairs do not establish Yaw–Pitch synergy.

| target_pair | unique_key | target_x_m | target_y_m | target_z_m | lift_m | mode | yaw_deg | pitch_deg | stored_arm_posture | success | arm_min_margin_rad | environment_clearance_mm | self_clearance_mm | failure_or_success_label | classification |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Yaw target | PHASE1\|R3_0.030\|R3\|0.030\|0.400\|LOCKED\|0 | 0.686481 | 0.227716 | 0.965 | 0.4 | LOCKED | 0 | 0 | False | False |  |  |  | GRASP_CONFIGURATION_IK_FAILURE | PURE_IK_ABSENCE |
| Yaw target | PHASE1\|R3_0.030\|R3\|0.030\|0.400\|YAW_ONLY\|0 | 0.686481 | 0.227716 | 0.965 | 0.4 | YAW_ONLY | -8 | 0 | True | True | 0.0528147 | 2.53962 | 0.65926 | LIFT_ACTUATED_EXTRACTION_SUCCESS | NOMINAL_FEASIBLE |
| Pitch target | PHASE1\|R3_0.020\|R3\|0.020\|0.350\|LOCKED\|0 | 0.682654 | 0.218478 | 0.965 | 0.35 | LOCKED | 0 | 0 | False | False |  |  |  | GRASP_CONFIGURATION_IK_FAILURE | PURE_IK_ABSENCE |
| Pitch target | PHASE1\|R3_0.020\|R3\|0.020\|0.350\|PITCH_ONLY\|0 | 0.682654 | 0.218478 | 0.965 | 0.35 | PITCH_ONLY | 0 | 18 | True | True | 0.436902 | 2.53961 | 0.65926 | LIFT_ACTUATED_EXTRACTION_SUCCESS | NOMINAL_FEASIBLE |

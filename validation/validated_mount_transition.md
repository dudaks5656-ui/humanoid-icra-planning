# Validated Arm Mount Alignment Transition

Date: 2026-08-12 (Asia/Seoul)  
Scope: `/home/openarm/humanoid_sim_ws` simulation workspace only

## Final authority

The user confirmed that the existing left/right arm mount XYZ values are the final physical shoulder alignment:

- user-defined physical shoulder alignment
- visually verified against the assembled platform
- do not replace with Inventor UCS comparison coordinates

The earlier hypothesis that the symmetric mount coordinates were erroneous is superseded. No Inventor UCS comparison coordinate was applied to the active robot model.

## Backups made before editing

Directory:

`/home/openarm/humanoid_sim_ws/validation/backups/validated_mount_transition_20260812`

| Backup | Pre-change SHA-256 |
|---|---|
| `provisional_arm_mount_alignment.yaml.before` | `14f66b46de79cc172d6c6e62f5b062d5bfcd4e44049a9e1bc57cd9826788cfa2` |
| `openarm_arms_adapter.xacro.before` | `6710754b8b75a5e82cbd06d4a483d81a5afe388864a14ff316af11b10782e337` |
| `humanoid_sim.srdf.before` | `083268f69bb0aa7ec6120cff141ac11185ee85f2c91c313cea07cc5116f61bdb` |

The original `provisional_arm_mount_alignment.yaml` remains present and unchanged as a historical source file. Its current hash remains `14f66b46de79cc172d6c6e62f5b062d5bfcd4e44049a9e1bc57cd9826788cfa2`.

## Validated configuration

Created:

`/home/openarm/humanoid_sim_ws/src/humanoid_sim_description/config/validated_arm_mount_alignment.yaml`

SHA-256: `6b3fb9b254a7d8438ab77eb747260a20c49d3ea6078a46de1e3ea1d444d527aa`

Values:

```yaml
validated_left_mount_xyz: [0.0435192366720, -0.0049688054576, 0.2020497164153]
validated_right_mount_xyz: [0.0435192366720, -0.0649688054576, 0.2020497164153]
```

`openarm_arms_adapter.xacro` now loads this validated file and keys. It no longer loads the provisional file. Only the configuration/property names and explanatory comments changed; no numeric mount value, adapter rotation, joint parent/child, mesh, limit, or collision origin changed.

Updated Xacro SHA-256: `b25f74e2b1facea2d106f7d58a43cc086eecc8744f5d71e92d02d6f9d683a280`

## Before/after URDF equality

Files:

- `/home/openarm/humanoid_sim_ws/validation/validated_mount_transition_before.urdf`
- `/home/openarm/humanoid_sim_ws/validation/validated_mount_transition_after.urdf`

Joint origin comparison:

| Joint | Before XYZ / RPY | After XYZ / RPY | Result |
|---|---|---|---|
| `waist_pitch_to_left_arm_mount_joint` | `0.043519236672 -0.0049688054576 0.2020497164153` / `0 0 0` | identical | PASS |
| `waist_pitch_to_right_arm_mount_joint` | `0.043519236672 -0.0649688054576 0.2020497164153` / `0 0 0` | identical | PASS |

The expanded URDF diff contains only the two explanatory comments changing from `PROVISIONAL_RVIZ_ALIGNMENT_ONLY` to the user-validated wording. After comments are removed, the canonical XML hashes are identical:

```text
fd89afcf94da8ad05ea3b1a7e8266ef382c31895c9c6db4a02287a59937e00c9  before
fd89afcf94da8ad05ea3b1a7e8266ef382c31895c9c6db4a02287a59937e00c9  after
```

The differing whole-file hashes are caused only by those comments:

```text
4379496406c2212b27c609a568920ee9463b72845aa1a19af3d69ff362b02567  before URDF
3f4bb743d35d468bb0166914318e5492de27bf4abd5a9b2bbbcc6ef843c255df  after URDF
```

## Build and static validation

- `humanoid_sim_description`: successfully rebuilt with `--symlink-install`
- Xacro expansion: PASS
- `xmllint --noout`: PASS
- `check_urdf`: PASS; root link remains `world`
- SRDF XML parse: PASS
- MoveIt/SRDF model load: PASS, 19 independent variables

The first post-edit Xacro attempt stopped because the new YAML had not yet been installed. No partial source change resulted. Rebuilding only `humanoid_sim_description` installed the new YAML, after which all equality checks passed.

No existing OpenArm hardware workspace or OpenArm control file was modified or built.


# Arm Mount Frame and Permanent-Collision Audit

Audit date: 2026-08-12 (Asia/Seoul)  
Scope: `/home/openarm/humanoid_sim_ws` simulation copy only  
Status: diagnostic comparison only; no description Xacro, SRDF, ACM, joint limit, mesh, or hardware file was changed.

## Coordinate-source candidates

| File | Modified (KST) | SHA-256 | Interpretation |
|---|---|---|---|
| `/home/openarm/Downloads/humanoid_sim_mesh_package_2026-08-12/cad_frame_transforms.yaml` | 2026-08-12 00:54:54 | `d42520f6ec0c6ee92d60f1238ee6373b2bddc26277f112fa036f25e6a5044840` | Delivered CAD transform source |
| `/home/openarm/Downloads/humanoid_sim_mesh_package_2026-08-12/ucs_matrices.json` | 2026-08-12 00:52:12 | `061a9a44a0800150893c23a53899c6de00890efa056822187e42a6512e3f1c1d` | Delivered Inventor UCS matrices |
| `/home/openarm/humanoid_sim_ws/src/humanoid_sim_description/config/cad_frame_transforms.yaml` | 2026-08-12 01:17:46 | `d42520f6ec0c6ee92d60f1238ee6373b2bddc26277f112fa036f25e6a5044840` | Byte-identical simulation copy |
| `/home/openarm/humanoid_sim_ws/src/humanoid_sim_description/config/ucs_matrices.json` | 2026-08-12 01:17:46 | `061a9a44a0800150893c23a53899c6de00890efa056822187e42a6512e3f1c1d` | Byte-identical simulation copy |
| `/home/openarm/humanoid_sim_ws/src/humanoid_description/config/cad_frame_transforms.yaml` | 2026-08-12 00:07:37 | `5cf940e4ab7cc2885439b9e154b327db87c56e0cc9117c00ebca7a475a34468e` | Old `NOT_RECEIVED` blank template; not a measurement source |

`inventor_measurements.yaml` and `module_frames.yaml` were not found under `/home/openarm`, `/media`, or `/mnt`.

## Current source selection

`openarm_arms_adapter.xacro` does not use the delivered CAD XYZ values for the mount joints. Lines 12–13 load `provisional_arm_mount_alignment.yaml`; lines 31–33 and 41–43 use its `provisional_left_mount_xyz` and `provisional_right_mount_xyz` entries. The comments mark both joints `PROVISIONAL_RVIZ_ALIGNMENT_ONLY`.

The same provisional YAML retains both data sets:

- delivered CAD left: `[0.0152640649629, 0.0025012666813, 0.1693164092888]`
- delivered CAD right: `[0.0063073195654, -0.1968363630487, 0.2211265056554]`
- current provisional left: `[0.0435192366720, -0.0049688054576, 0.2020497164153]`
- current provisional right: `[0.0435192366720, -0.0649688054576, 0.2020497164153]`

## Kinematic chain

```text
waist_pitch_link
  -> waist_pitch_to_left_arm_mount_joint [fixed]
  -> left_arm_mount_link
  -> left_arm_mount_to_openarm_joint [fixed, rpy=-1.5708 0 0]
  -> openarm_left_link0

waist_pitch_link
  -> waist_pitch_to_right_arm_mount_joint [fixed]
  -> right_arm_mount_link
  -> right_arm_mount_to_openarm_joint [fixed, rpy=+1.5708 0 0]
  -> openarm_right_link0
```

There is no movable joint between `waist_pitch_link` and either link0. The adapter rotations are child-frame orientations applied after each mount translation; they do not rotate or explain the XYZ stored in the parent `waist_pitch_link` frame.

## Inventor matrix verification

All relevant Inventor UCS matrices have the same global rotation:

```text
R = [ 0 -1  0
      1  0  0
      0  0  1 ]
```

For each arm, the relative transform is:

```text
T_pitch_mount = inverse(T_global_pitch) * T_global_mount
```

The translation calculation gives:

| Side | Global mount − global pitch (cm) | `Rᵀ·delta` (cm) | Result (m) |
|---|---|---|---|
| Left | `[-0.25012666813, 1.52640649629, 16.93164092888]` | `[1.52640649629, 0.25012666813, 16.93164092888]` | `[0.0152640649629, 0.0025012666813, 0.1693164092888]` |
| Right | `[19.68363630487, 0.63073195654, 22.11265056554]` | `[0.63073195654, -19.68363630487, 22.11265056554]` | `[0.0063073195654, -0.1968363630487, 0.2211265056554]` |

These values exactly reproduce `cad_frame_transforms.yaml` to displayed precision.

Current minus CAD XYZ:

- left: `[+0.0282551717091, -0.0074700721389, +0.0327333071265]` m
- right: `[+0.0372119171066, +0.1318675575911, -0.0190767892401]` m

The current right-minus-left vector is `[0, -0.0600000000000, 0]` m. The CAD right-minus-left vector is `[-0.0089567453975, -0.1993376297300, 0.0518100963666]` m. Their lengths differ, so a common rigid rotation or translation cannot map the current pair to the CAD pair. The ±90° adapter rotations also cannot account for this difference because the mount XYZ is defined before those rotations.

The supported classification is **temporary symmetric coordinates remain in the active Xacro path**. The delivered YAML and JSON are internally consistent and byte-identical between the download and simulation-copy locations. Physical CAD revision confirmation remains necessary before changing the active model.

## Temporary URDF comparison

Two new validation-only URDFs were generated. The current model is an unmodified Xacro expansion. The Inventor model differs only in the two mount-joint XYZ values; adapter rotations, meshes, scales, limits, and SRDF are identical.

- `/home/openarm/humanoid_sim_ws/validation/arm_mount_current.urdf`
- `/home/openarm/humanoid_sim_ws/validation/arm_mount_inventor.urdf`

Both pass `xmllint --noout` and `check_urdf`, with root `world` and identical link trees.

FCL zero-state comparison:

| Variant | Pair | Contacts | Max depth (m) | Mean depth (m) | AABB overlap XYZ (m) |
|---|---|---:|---:|---:|---|
| current provisional | Pitch ↔ left link0 | 346 | 0.0380000259 | 0.00128103265 | `[0.0600000, 0.00666020, 0.12200004]` |
| current provisional | Pitch ↔ right link0 | 0 | 0 | 0 | `[0.0600000, 0.01150047, 0.12200004]` |
| Inventor YAML | Pitch ↔ left link0 | 0 | 0 | 0 | `[0.05274485, 0, 0.12200004]` |
| Inventor YAML | Pitch ↔ right link0 | 0 | 0 | 0 | `[0.04378811, 0, 0.10292325]` |

An AABB overlap alone does not prove triangle-mesh contact, as shown by the current right pair. The 38 mm left penetration disappears when only the mount XYZ is replaced by the delivered Inventor values. It therefore must not be classified as an intended fixed mounting contact.

Detailed rows are in `arm_mount_frame_comparison.csv`.

## Left negative-Y mesh scale

The OpenArm v1.0 macro explicitly assigns `reflect=-1` to non-right arms and `reflect=+1` to right arms. `openarm_macro.xacro` applies the same reflection to visual and collision mesh scales.

| Side | Visual scale | Collision scale |
|---|---|---|
| Left link0 | `0.001 -0.001 0.001` | `0.001 -0.001 0.001` |
| Right link0 | `0.001 0.001 0.001` | `0.001 0.001 0.001` |

Both use the same `link0_symp.stl`. The file has 804 triangles. Positive and negative-Y scaled meshes have identical metric extents:

```text
[0.0979364891, 0.1220000000, 0.0624999990] m
```

Their absolute signed volume is identical (`0.000477135279 m^3`); the sign reverses because a single-axis reflection reverses triangle orientation. FCL still loads and tests the reflected mesh, and the Inventor-position left model produces zero contact. There is no evidence that scale magnitude, mesh size, or FCL failure caused the 38 mm collision. Winding-dependent contact-normal orientation should still be treated cautiously, but the mesh was not modified in this audit.

## Finger sweep

Each independent `finger_joint1` was sampled at 21 points from 0.0000 to 0.0440 m. `finger_joint2` followed with mimic factor 1 and offset 0.

Both sides show the same collision interval at this resolution:

- collision: q = 0.0000, 0.0022, 0.0044, 0.0066 m
- no collision: q = 0.0088 through 0.0440 m
- zero-state maximum depth: 0.00986640692 m
- maximum observed depth: about 0.01088605764 m at q = 0.0022 m

No OpenArm SRDF named state was found that proves whether q=0 or q=0.044 is the physical `open` or `closed` pose. Numerically this is a lower-limit-end collision, not an all-range permanent collision. Because the collision disappears well inside the existing range, it is a joint-limit/geometry/origin investigation candidate; it is not yet justified for ACM disabling.

Detailed rows are in `finger_collision_sweep.csv`.

## Integrity hashes

```text
e0bcfb64859d94203eeec45f823dec8d238d72bec6809d81c2c99091942be4e4  humanoid_sim.urdf.xacro
6710754b8b75a5e82cbd06d4a483d81a5afe388864a14ff316af11b10782e337  openarm_arms_adapter.xacro
14f66b46de79cc172d6c6e62f5b062d5bfcd4e44049a9e1bc57cd9826788cfa2  provisional_arm_mount_alignment.yaml
083268f69bb0aa7ec6120cff141ac11185ee85f2c91c313cea07cc5116f61bdb  humanoid_sim.srdf
baf52578e1d9e6225f3818cae82b6074a0b948d3cef8e9a3e6dfafca78507590  link0_symp.stl
4379496406c2212b27c609a568920ee9463b72845aa1a19af3d69ff362b02567  arm_mount_current.urdf
cafeec76fb4793ad23b2095edc455693f8f668313d3980a8cb87744668c57430  arm_mount_inventor.urdf
2e4142f99c11deb9e27ee7bd9c218853ae455ceaf9ad171abc165019ce9f0bde  arm_mount_frame_comparison.csv
6b8ccefd9e905c1f1efcc8f6e3a4d2e77be73d3d6ab7c560e2cf6c6459431050  finger_collision_sweep.csv
```

No stop signal or reload request was sent to the existing RViz/MoveIt session during this audit. At the final status check, however, its recorded PIDs no longer existed and the ROS_DOMAIN_ID=42 graph was empty. The launch log contains only the original process-start records and no recorded shutdown cause. Neither temporary URDF was loaded into RViz.

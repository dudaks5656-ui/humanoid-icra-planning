# OpenArm Description Source Provenance

## Source and model status

- Original package: `/home/openarm/openarm/leader/source/openarm/repos/openarm_description`
- Copied package: `/home/openarm/humanoid_sim_ws/src/openarm_description`
- Copy timestamp: `2026-08-11T14:42:29+09:00` (Asia/Seoul)
- `robot_model_status`: `provisional_v10`
- Provisional model designation: OpenArm `v10` / `v1.0`
- This designation is **not** a final identification of the physical robot. The hardware model must be confirmed by the user before producing final research data.

## Copied scope

The following description-only package content was copied without modification:

- `package.xml`
- `CMakeLists.txt`
- `assets/` (Xacro, URDF, meshes, material/description data, and description configuration)
- `launch/` (`display_openarm.launch.py`)
- `rviz/` (`arm_only.rviz`, `bimanual.rviz`)

The source package has no `scripts/` directory, so no scripts were copied. The complete copied scope contains 153 regular files. No control repository, hardware driver, or existing `build/`, `install/`, or `log/` directory was copied.

The protected source package and all other existing OpenArm hardware workspaces were read only. No original file was modified, moved, deleted, or built.

## Integrity result

- Recursive byte comparison for `package.xml`, `CMakeLists.txt`, `assets/`, `launch/`, and `rviz/`: **MATCH**
- SHA-256 comparison set: every copied Xacro and mesh file (`.xacro`, `.stl`, `.dae`), 92 files total
- SHA-256 mismatch count: **0**
- Broken symbolic links in copied package: **0**

The tables below record the provisional v10 expansion's principal Xacro files and active mesh assets. Because each pair matched, the same SHA-256 value applies to both the original and copied file.

### Principal v10 Xacro files

| Relative path | Original SHA-256 | Copied SHA-256 | Result |
|---|---|---|---|
| `assets/robot/openarm_v1.0/urdf/openarm_v10.urdf.xacro` | `0ae58c2baafb2ec6ba63d410a9fe4632a9d50f873a302f48df3ed40f070f5056` | `0ae58c2baafb2ec6ba63d410a9fe4632a9d50f873a302f48df3ed40f070f5056` | MATCH |
| `assets/robot/openarm_v1.0/urdf/robot/openarm_robot.xacro` | `8a33c8e4ce26e464a202d860ad57706b187da85800ae40070c11d0e999f08c7f` | `8a33c8e4ce26e464a202d860ad57706b187da85800ae40070c11d0e999f08c7f` | MATCH |
| `assets/robot/openarm_v1.0/urdf/arm/openarm_arm.xacro` | `4660227cea93f164959197770ab3a7b32e13366457cf35a19542ef6ef11aadf9` | `4660227cea93f164959197770ab3a7b32e13366457cf35a19542ef6ef11aadf9` | MATCH |
| `assets/robot/openarm_v1.0/urdf/arm/openarm_macro.xacro` | `eea9089f4e79cc22d2ee90c7d437c7f8636de6b88abcb2553a07b05a7dbfd5bd` | `eea9089f4e79cc22d2ee90c7d437c7f8636de6b88abcb2553a07b05a7dbfd5bd` | MATCH |
| `assets/robot/openarm_v1.0/urdf/body/openarm_body.xacro` | `5883ddd08053f9c79f84c7c8f05d4b093a571689422649ba18d93bec31f5cda5` | `5883ddd08053f9c79f84c7c8f05d4b093a571689422649ba18d93bec31f5cda5` | MATCH |
| `assets/robot/openarm_v1.0/urdf/body/openarm_body_macro.xacro` | `e71e788e3292435389d944995f0a1cf8ea43535d02442193fe099002ff9e6170` | `e71e788e3292435389d944995f0a1cf8ea43535d02442193fe099002ff9e6170` | MATCH |
| `assets/robot/openarm_v1.0/urdf/ee/ee_dispatcher.xacro` | `7de2e2d9b63e80f51c56f0adfc0d8860cd5aaf9bc198387ba73a0e0f329eda10` | `7de2e2d9b63e80f51c56f0adfc0d8860cd5aaf9bc198387ba73a0e0f329eda10` | MATCH |
| `assets/robot/openarm_v1.0/urdf/ee/openarm_ee_macro.xacro` | `7b7b1b98ecd8bc6466e1d3da5ead2c7eebe78a392cb4115f7f7c5914d7d123ac` | `7b7b1b98ecd8bc6466e1d3da5ead2c7eebe78a392cb4115f7f7c5914d7d123ac` | MATCH |
| `assets/robot/openarm_v1.0/urdf/ee/parallel_link/openarm_parallel_gripper.xacro` | `6badf44dabe426fc15e73f46607bcb0221a8bab67256db7c9731a988abc4d319` | `6badf44dabe426fc15e73f46607bcb0221a8bab67256db7c9731a988abc4d319` | MATCH |
| `assets/robot/openarm_v1.0/urdf/ros2_control/openarm.ros2_control.xacro` | `f11624796d2de0710dbb63c4cb41783e84fd942605bd254745335179783d6f32` | `f11624796d2de0710dbb63c4cb41783e84fd942605bd254745335179783d6f32` | MATCH |
| `assets/robot/openarm_v1.0/urdf/ros2_control/openarm.bimanual.ros2_control.xacro` | `49a27fd36296045341c8c10f61936ba84942f86a8bfc2a00c7a21e32f50a512e` | `49a27fd36296045341c8c10f61936ba84942f86a8bfc2a00c7a21e32f50a512e` | MATCH |

### Active v10 mesh assets

| Relative path | Original SHA-256 | Copied SHA-256 | Result |
|---|---|---|---|
| `assets/end_effector/parallel_link/meshes/collision/finger.stl` | `8e96e1314618cf434908f70df78f68dd2b049c03538964e8d41fc99abe41564d` | `8e96e1314618cf434908f70df78f68dd2b049c03538964e8d41fc99abe41564d` | MATCH |
| `assets/end_effector/parallel_link/meshes/visual/finger.dae` | `64492f6a80a551a7a53e3f769facc42ee4a9bce1634ff63300a9d7f5b3628cb0` | `64492f6a80a551a7a53e3f769facc42ee4a9bce1634ff63300a9d7f5b3628cb0` | MATCH |
| `assets/robot/openarm_v1.0/mesh/arm/collision/link0_symp.stl` | `baf52578e1d9e6225f3818cae82b6074a0b948d3cef8e9a3e6dfafca78507590` | `baf52578e1d9e6225f3818cae82b6074a0b948d3cef8e9a3e6dfafca78507590` | MATCH |
| `assets/robot/openarm_v1.0/mesh/arm/collision/link1_symp.stl` | `066113d13d5cc85098609003bc7ebb73c570015350877f5ed7162ef1b6601852` | `066113d13d5cc85098609003bc7ebb73c570015350877f5ed7162ef1b6601852` | MATCH |
| `assets/robot/openarm_v1.0/mesh/arm/collision/link2_symp.stl` | `382ab32e4ae0880e8a1512e7a6ca6ce1f478a6c125db4efa977429ffb1d6b02a` | `382ab32e4ae0880e8a1512e7a6ca6ce1f478a6c125db4efa977429ffb1d6b02a` | MATCH |
| `assets/robot/openarm_v1.0/mesh/arm/collision/link3_symp.stl` | `00c908cefab152c00416a570a48bf9aafed1549085f19ff2d882dc3f355d9f59` | `00c908cefab152c00416a570a48bf9aafed1549085f19ff2d882dc3f355d9f59` | MATCH |
| `assets/robot/openarm_v1.0/mesh/arm/collision/link4_symp.stl` | `b54883b8c7c96268a68a5879f95998a53ad0b0c4fe74325fad63a6caef669c73` | `b54883b8c7c96268a68a5879f95998a53ad0b0c4fe74325fad63a6caef669c73` | MATCH |
| `assets/robot/openarm_v1.0/mesh/arm/collision/link5_symp.stl` | `678a2802906eff7b45a836d2f34a2d8e51def50b6599376968f888e05c72739e` | `678a2802906eff7b45a836d2f34a2d8e51def50b6599376968f888e05c72739e` | MATCH |
| `assets/robot/openarm_v1.0/mesh/arm/collision/link6_symp.stl` | `95529bec23733476dfdbbb266c7db0d25a473a568de73c8337a82440fe4a9ac3` | `95529bec23733476dfdbbb266c7db0d25a473a568de73c8337a82440fe4a9ac3` | MATCH |
| `assets/robot/openarm_v1.0/mesh/arm/collision/link7_symp.stl` | `434f207f21f75f5f0bd604e390b8e5bc7b62b619265222846770e06b3f9b5cfb` | `434f207f21f75f5f0bd604e390b8e5bc7b62b619265222846770e06b3f9b5cfb` | MATCH |
| `assets/robot/openarm_v1.0/mesh/arm/visual/link0.dae` | `29037f2e51d6b34dbbdbd1eecb557abd900aa1897be4d4a30a7114e649f987b9` | `29037f2e51d6b34dbbdbd1eecb557abd900aa1897be4d4a30a7114e649f987b9` | MATCH |
| `assets/robot/openarm_v1.0/mesh/arm/visual/link1.dae` | `c25937f901a4c15730927e08bf5fd26a072e801c529f0bd9678ba9758330647d` | `c25937f901a4c15730927e08bf5fd26a072e801c529f0bd9678ba9758330647d` | MATCH |
| `assets/robot/openarm_v1.0/mesh/arm/visual/link2.dae` | `5d2a1ec857a22eaa180034e2dcd989c54611abab6ce6659eb4999570c34cc124` | `5d2a1ec857a22eaa180034e2dcd989c54611abab6ce6659eb4999570c34cc124` | MATCH |
| `assets/robot/openarm_v1.0/mesh/arm/visual/link3.dae` | `97d73c4bae3f3f7570bf3e28306da719aa99a6bec662060c9e358e5625b11cce` | `97d73c4bae3f3f7570bf3e28306da719aa99a6bec662060c9e358e5625b11cce` | MATCH |
| `assets/robot/openarm_v1.0/mesh/arm/visual/link4.dae` | `59eb1cbf44b1250ad9533176b30084179f9164fcb998ee2623d89b48968ec426` | `59eb1cbf44b1250ad9533176b30084179f9164fcb998ee2623d89b48968ec426` | MATCH |
| `assets/robot/openarm_v1.0/mesh/arm/visual/link5.dae` | `96fd3516df16a772a84c268529e5b53b05e6df07da6ccfe850bc091eba947156` | `96fd3516df16a772a84c268529e5b53b05e6df07da6ccfe850bc091eba947156` | MATCH |
| `assets/robot/openarm_v1.0/mesh/arm/visual/link6.dae` | `adcd37741f4e223bd6fe9925625e84191b525c9d27f151f9178b23989de05459` | `adcd37741f4e223bd6fe9925625e84191b525c9d27f151f9178b23989de05459` | MATCH |
| `assets/robot/openarm_v1.0/mesh/arm/visual/link7.dae` | `e0e1d59b1e09c5a73198cdf175dbb94844073ec067d4b491dcab8a9e1a7faade` | `e0e1d59b1e09c5a73198cdf175dbb94844073ec067d4b491dcab8a9e1a7faade` | MATCH |
| `assets/robot/openarm_v1.0/mesh/body/collision/body_link0_symp.stl` | `6c18bbf7e86b03e3faf802e61e8eb438b38dcbcf146d97cffe6e808c65e9a72a` | `6c18bbf7e86b03e3faf802e61e8eb438b38dcbcf146d97cffe6e808c65e9a72a` | MATCH |
| `assets/robot/openarm_v1.0/mesh/body/visual/body_link0.dae` | `a05f59379dcf694e5ff336b1ee3a47ea504a2425a2fa774af1b14e096d9a239a` | `a05f59379dcf694e5ff336b1ee3a47ea504a2425a2fa774af1b14e096d9a239a` | MATCH |

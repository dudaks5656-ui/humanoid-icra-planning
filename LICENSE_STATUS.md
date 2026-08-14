# License and repository file status

This document is an asset-classification record, not legal advice and not a grant of rights. The initial GitHub repository must remain private. No top-level project license is asserted until ownership and third-party obligations are confirmed.

## GitHub commit possible

The following classes are intended for the initial private commit after secret, size, and ignore checks:

- Root handoff/workflow documentation and helper shell scripts.
- Project-authored C++/Python code, CMake files, package manifests, launch files, RViz configuration, Xacro/URDF/SRDF, and YAML configuration.
- OpenArm-derived non-mesh description/configuration files, with provenance retained and public redistribution still subject to the review below.
- Selected small validation CSV/YAML/TXT artifacts, SHA-256 manifests, and audit Markdown reports.
- `ICRA_PROJECT_HANDOFF.md` is mandatory in every handoff checkout.

Some committed source and audit files contain `/home/openarm` absolute paths or historical host/path text. The initial repository is private and preserves these files unchanged. Future portability work should replace runtime output paths in a separately reviewed change without rewriting historical evidence.

## Local preservation only

- `build/`, `install/`, and `log/`.
- `validation/backups/`, launch/runtime logs, Python caches, core dumps, RViz screenshots carrying local UI context, bags, and databases.
- Large raw IK/reachability/repeat CSVs excluded by `.gitignore`, including the two approximately 8–9 MB top-open reachability files, `differential_internal_ik.csv`, `ik_multistart_audit.csv`, `*internal_ik.csv`, and `*repeat_trials*.csv`.
- Local ROS logs under `/home/openarm/.ros`, which are outside this workspace.
- Host-specific provenance/log snapshots containing an original Windows user path, workstation hostname, or full ROS log path, plus superseded `before_session_status_correction` reports.

These files are not deleted. They remain on the Ubuntu workstation and can be summarized into small reviewable artifacts.

## OpenArm-derived files

`src/openarm_description` was copied from an existing OpenArm description workspace. `SOURCE_PROVENANCE.md` records the source, copy scope, and integrity hashes. Source headers and `package.xml` refer to Apache-2.0, but this workspace copy does not contain a top-level upstream LICENSE/NOTICE file.

- Non-mesh Xacro/config/source files may be included in the initial **private** research handoff with provenance preserved.
- Before any public release, obtain and include the exact upstream license text and notices, verify the upstream repository/revision, and confirm that every copied asset is covered.
- Do not infer that an Apache header on code automatically licenses mesh/CAD assets.

## Project CAD and mesh assets

The humanoid STL files were exported from project CAD, and the OpenArm description contains STL/DAE assets. Ownership and redistribution permission are not established by the current workspace.

All `*.stl`, `*.dae`, and `*.obj` files, plus Inventor/STEP source formats (`*.iam`, `*.ipt`, `*.step`, `*.stp`), are therefore excluded from Git by default. This includes both project-created humanoid meshes and copied OpenArm meshes. They must remain local until written permission or an applicable asset license is recorded.

Largest excluded source assets found during the 2026-08-14 audit include:

| File | Size |
|---|---:|
| `src/humanoid_sim_description/meshes/amr_base.stl` | 20,506,384 bytes |
| `src/humanoid_sim_description/meshes/lift_fixed_structure.stl` | 13,917,584 bytes |
| OpenArm v1.0/v2.0 `body_link0.dae` | 10,783,948 bytes each |
| OpenArm v1.0 `link5.dae` | 9,034,473 bytes |

If redistribution is later approved, use Git LFS or a separately versioned asset package rather than silently adding binaries to normal Git history.

## Credential and personal-data assessment

The pre-commit audit found no `.env`, key/certificate, Git credential, or token-named file and no common private-key, GitHub token, AWS access-key, bearer-token, password, API-key, access-token, or client-secret content pattern outside generated trees and binary meshes.

Potentially identifying material still exists in historical evidence:

- `/home/openarm` absolute paths;
- a Windows OneDrive/CAD source path;
- ROS log paths and a workstation hostname in audit reports;
- package maintainer/placeholder addresses `openarm@example.com`, `openarm@todo.todo`, and `openarm_dev@enactic.ai`; none was identified as a user's personal email in this audit.

Runtime/build logs, screenshots, backup trees, credential/environment files, and local browser/session material must not be committed. Private repository visibility does not replace this check.

## File classification summary

| Classification | Initial policy |
|---|---|
| GitHub commit possible | Source, config, launch, helper scripts, handoff/workflow docs, selected small validation/audits |
| Local only | Generated trees, backups, logs, caches, core dumps, bags/databases, screenshots, large raw experiments |
| License confirmation required | All mesh/CAD assets; OpenArm-derived files before public redistribution |
| Large files | Meshes listed above and raw reachability/IK CSVs; excluded |
| Credential possibility | Logs, environment files, key files, browser/session data, host-specific evidence; excluded or reviewed |

## Future public release

Public release is **not currently approved**. It requires, at minimum:

1. ownership and contributor approval for project-authored code;
2. an explicit project LICENSE;
3. upstream OpenArm license/NOTICE and revision verification;
4. written mesh/CAD redistribution determination;
5. removal or parameterization of unnecessary personal absolute paths;
6. a fresh credential/PII scan of the exact public commit;
7. a decision on whether validation data is source, release data, or external archival data.

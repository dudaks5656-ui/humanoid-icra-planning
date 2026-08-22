#!/usr/bin/env python3
"""Create and immediately verify the collision-free FK workspace SHA-256 manifest."""
import glob
import hashlib
import os

WS="/home/openarm/humanoid_sim_ws"
MANIFEST=f"{WS}/validation/collision_free_fk_workspace_manifest_sha256.txt"
if os.path.exists(MANIFEST):
    raise RuntimeError(f"Refusing to overwrite: {MANIFEST}")
paths=[]
for path in (
    f"{WS}/src/humanoid_sim_description/urdf/openarm_arms_adapter.xacro",
    f"{WS}/validation/grasp_tcp_runtime_34p5mm_consistency_audit.md",
):
    if not os.path.isfile(path):
        raise RuntimeError(f"Required runtime TCP artifact missing: {path}")
    paths.append(path)
for pattern in (
    f"{WS}/src/collision_free_fk_workspace_analysis/**/*",
    f"{WS}/validation/collision_free_fk_workspace_*",
    f"{WS}/presentation/collision_free_fk_workspace_*",
):
    paths.extend(path for path in glob.glob(pattern,recursive=True) if os.path.isfile(path))
paths=sorted(set(paths)-{MANIFEST})
if not paths:
    raise RuntimeError("No workspace outputs found")
def digest(path):
    h=hashlib.sha256()
    with open(path,"rb") as stream:
        for block in iter(lambda:stream.read(1024*1024),b""):h.update(block)
    return h.hexdigest()
with open(MANIFEST,"x",encoding="utf-8") as stream:
    for path in paths:stream.write(f"{digest(path)}  {os.path.relpath(path,WS)}\n")
for line in open(MANIFEST,encoding="utf-8"):
    expected,relative=line.rstrip().split("  ",1)
    actual=digest(f"{WS}/{relative}")
    if actual!=expected:raise RuntimeError(f"Manifest mismatch: {relative}")
print(f"MANIFEST PASS files={len(paths)} path={MANIFEST}")

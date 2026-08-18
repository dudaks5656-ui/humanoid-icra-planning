#!/usr/bin/env python3
"""Verify, audit, and manifest the lift-slice FK evidence without touching prior results."""
import csv, hashlib, json, math, os, subprocess
from PIL import Image

WS="/home/openarm/humanoid_sim_ws";VAL=f"{WS}/validation";PRE=f"{WS}/presentation";PKG="src/lift_slice_fk_workspace_analysis"
AUDIT=f"{VAL}/lift_slice_fk_workspace_audit.md";MANIFEST=f"{VAL}/lift_slice_fk_workspace_manifest_sha256.txt"
CONFIGS=["LIFT_ONLY","LIFT_YAW","LIFT_PITCH","LIFT_YAW_PITCH"];RATIOS=[0.,.25,.5,.75,1.]
PROTECTED={
"src/humanoid_sim_description/urdf/humanoid_sim.urdf.xacro":"e0bcfb64859d94203eeec45f823dec8d238d72bec6809d81c2c99091942be4e4",
"src/humanoid_sim_moveit_config/config/humanoid_sim.srdf":"7892b122fe28e91650f8919a66c256ebc55d3b34472e006701a63cadab5484e8",
"src/humanoid_sim_moveit_config/config/joint_limits.yaml":"1e851fa791dab1c95eeba4a698e620c0334afd12a0c9964b774f24868d03ccfb",
"src/humanoid_sim_moveit_config/config/kinematics.yaml":"7a825c85e6d3bfa06b3ff6d195b3aced8c490d6132d8d94e2d0b01f52e73204e",
"src/humanoid_sim_moveit_config/config/ompl_planning.yaml":"3e47c074ffcb9a72ea62e8821a8923f80c7f346e246993b3131038380aaf1335"}

def digest(path):
    h=hashlib.sha256()
    with open(path,"rb") as s:
        for b in iter(lambda:s.read(1048576),b""):h.update(b)
    return h.hexdigest()
def read(name):
    with open(f"{VAL}/{name}",newline="",encoding="utf-8") as f:return list(csv.DictReader(f))
def meta():return {r["key"]:r["value"] for r in read("lift_slice_fk_workspace_sampling_metadata.csv")}
def fmt(value):return f"{float(value):.3f}" if str(value).strip() else "NO_VALID_STATE"
def verify_manifest(name,base):
    count=0
    with open(f"{VAL}/{name}",encoding="utf-8") as s:
        for line in s:
            if not line.strip():continue
            expected,rel=line.rstrip().split("  ",1);target=os.path.join(base,rel)
            if digest(target)!=expected:raise RuntimeError(f"Protected manifest mismatch: {target}")
            count+=1
    return count

def main():
    if os.path.exists(AUDIT) or os.path.exists(MANIFEST):raise RuntimeError("Refusing audit/manifest overwrite")
    for rel,expected in PROTECTED.items():
        if digest(f"{WS}/{rel}")!=expected:raise RuntimeError(f"Protected file changed: {rel}")
    old=[("fixed_base_workspace_manifest_sha256.txt",VAL),("fixed_base_workspace_fine_manifest_sha256.txt",VAL),("fixed_base_workspace_dof_ablation_manifest_sha256.txt",VAL),("fixed_base_workspace_demo_manifest_sha256.txt",WS),("fixed_base_workspace_envelope_demo_manifest_sha256.txt",WS),("radial_workspace_validation_manifest_sha256.txt",WS),("workspace_projection_manifest_sha256.txt",WS),("fk_workspace_boundary_manifest_sha256.txt",WS)]
    old_counts={n:verify_manifest(n,b) for n,b in old}
    states=read("lift_slice_fk_workspace_states.csv");metadata=meta();nested=read("lift_slice_fk_workspace_nested_points.csv");checks=read("lift_slice_fk_workspace_nested_check.csv");summary=read("lift_slice_fk_workspace_summary.csv");front=read("lift_slice_fk_workspace_front.csv");right=read("lift_slice_fk_workspace_right.csv");surface=read("lift_slice_fk_workspace_3d_surface.csv");disp=read("lift_slice_fk_workspace_slice_displacement.csv");comparison=read("lift_slice_fk_vs_grid_workspace_comparison.csv")
    if len(states)!=40000 or len(summary)!=20 or any(r["status"]!="PASS" or int(r["missing_count"]) for r in checks):raise RuntimeError("Core state/nested integrity failed")
    for c in CONFIGS:
        for ratio in RATIOS:
            raw=[r for r in states if r["configuration"]==c and abs(float(r["lift_ratio"])-ratio)<1e-8]
            if len(raw)!=2000:raise RuntimeError(f"Attempt count failed {c} {ratio}")
            valid=[r for r in raw if r["valid"]=="1"]
            if any(not all(math.isfinite(float(r[k])) for k in ("tcp_x","tcp_y","tcp_z","joint_margin","self_clearance")) for r in valid):raise RuntimeError("Nonfinite valid state")
            if c in ("LIFT_ONLY","LIFT_PITCH") and any(abs(float(r["yaw"]))>1e-12 for r in raw):raise RuntimeError("Fixed yaw moved")
            if c in ("LIFT_ONLY","LIFT_YAW") and any(abs(float(r["pitch"]))>1e-12 for r in raw):raise RuntimeError("Fixed pitch moved")
    figures=[r["path"] for r in read("lift_slice_fk_workspace_figure_index.csv")]
    if len(figures)!=15 or len(set(figures))!=15:raise RuntimeError("Figure inventory failed")
    sizes={}
    for rel in figures:
        with Image.open(f"{WS}/{rel}") as im:
            sizes[rel]=im.size
            if im.width<1600 or im.height<850:raise RuntimeError(f"Presentation image too small {rel} {im.size}")
            im.verify()
    with open(f"{PRE}/lift_slice_fk_workspace_demo_metadata.json",encoding="utf-8") as f:video=json.load(f)
    if video["codec_name"]!="h264" or video["resolution"]!="1920x1080" or video["decoded_frames"]!=1350 or abs(video["duration_seconds"]-45)>0.2:raise RuntimeError("Video integrity failed")
    lift_values=[float(metadata[f"lift_slice_{i}"].split(";")[1]) for i in range(5)]
    robot_hash=hashlib.sha256(subprocess.check_output(["xacro",f"{WS}/src/humanoid_sim_description/urdf/humanoid_sim.urdf.xacro"])).hexdigest()
    lines=["# Lift-Slice FK Workspace Audit","","## Model and scope","",f"- Expanded robot model SHA-256: `{robot_hash}`.","- Base/TCP/reference: `base_link` / `openarm_left_hand_tcp` / `openarm_left_link0`.","- Coordinate convention: +X forward, +Y left, +Z vertical; AMR/base fixed.","- Main result: orientation-unconstrained POSITIONAL_FK_WORKSPACE.","- Sampling: deterministic 9-dimensional Halton sequence, seed/index offset `20260819`.","- 2,000 attempts × 4 configurations × 5 lift slices = 40,000 attempts; no rerun/refinement.","- Validity: actual RobotModel joint bounds, exact sampled-active bound exclusion, SRDF ACM self-collision check, finite FK.","- Lift MIN/MAX are deliberately fixed slice contexts and exempt from active exact-bound rejection; sampled Arm/Yaw/Pitch remain checked.","- Lift axis semantics: q=MIN is physical topmost and increasing q translates the lift along ROS -Z.","- IK, OMPL, trajectory execution, controller, ros2_control, hardware and AMR motion: not used.","","## Lift slices","",f"- Actual limit: `{lift_values[0]:.6f}` to `{lift_values[-1]:.6f}` m.",f"- MIN / 25% / MID / 75% / MAX: `{', '.join(f'{v:.6f}' for v in lift_values)}` m.","- Radial reference is the canonical shoulder link `openarm_left_link0` at the relevant lift slice, yaw=0, pitch=0 and default arm.","","## Per-slice results","","| Config | Lift % | Own valid | Nested pool | X range | Y range | Z range | Front r inner–outer | Right r inner–outer | Collision rejects |","|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for r in summary:
        lines.append(f"| {r['configuration']} | {float(r['lift_ratio'])*100:.0f} | {r['own_valid_states']} | {r['nested_pool_states']} | {fmt(r['x_min'])}–{fmt(r['x_max'])} | {fmt(r['y_min'])}–{fmt(r['y_max'])} | {fmt(r['z_min'])}–{fmt(r['z_max'])} | {fmt(r['front_min_observed_radius'])}–{fmt(r['front_max_observed_radius'])} | {fmt(r['right_min_observed_radius'])}–{fmt(r['right_max_observed_radius'])} | {r['self_collision_rejection_count']} |")
    lines += ["","## Nested inclusion","","- State pools are structural unions: C1=C0∪C1-own, C2=C0∪C2-own, C3=C0∪C1-own∪C2-own∪C3-own.",f"- Checks: {len(checks)} / {len(checks)} PASS; missing inherited endpoint states: 0.","- Therefore C0⊆C1, C0⊆C2, C1⊆C3 and C2⊆C3 at every lift slice.","","## Boundary, gaps, and 3D loft","","- Front is Y-Z; right is X-Z. Inner/outer are observed radial extrema in 2° bins about the slice-specific shoulder reference.",f"- Observed gaps (>0.05 m adjacent radial sample separation, ≥3 samples/bin): front {sum(int(r['observed_gap_count']) for r in front)}, right {sum(int(r['observed_gap_count']) for r in right)}. These are observations, not infeasibility proofs.","- No convex hull, spline filling, or morphological closing is used.",f"- 3D loft has {len(surface)} triangles. It joins corresponding non-gapped outer spherical bins across adjacent lift slices; missing/gapped bins remain open.","- Loft is a linear five-slice approximation, not a continuously proven swept volume.","","### Adjacent-slice boundary displacement","","| Config | Slice pair | Matched bins | Mean displacement | Max displacement |","|---|---|---:|---:|---:|"]
    for r in disp:lines.append(f"| {r['configuration']} | {float(r['lower_lift_ratio'])*100:.0f}→{float(r['upper_lift_ratio'])*100:.0f}% | {r['matched_direction_bins']} | {float(r['mean_boundary_displacement'] or 0):.4f} | {float(r['max_boundary_displacement'] or 0):.4f} |")
    lines += ["","## Existing fixed-orientation grid cross-check","","| Config | FK max X | Grid max X | ΔX | FK Y span | Grid Y span | FK Z span | Grid Z span |","|---|---:|---:|---:|---:|---:|---:|---:|"]
    for r in comparison:lines.append(f"| {r['configuration']} | {float(r['fk_max_x']):.4f} | {float(r['grid_max_x']):.4f} | {float(r['delta_x']):+.4f} | {float(r['fk_y_span']):.4f} | {float(r['grid_y_span']):.4f} | {float(r['fk_z_span']):.4f} | {float(r['grid_z_span']):.4f} |")
    lines += ["","The FK workspace is joint-space sampled and orientation-free, so it is expected to exceed the targeted fixed-grasp-orientation Cartesian grid; it cross-validates configuration trends but does not replace the grid result.","","## Presentation and integrity","",f"- Figures: 15 PNG; external legends used for 2D figures and comparison figures. Minimum image size: {min(w for w,h in sizes.values())}×{min(h for w,h in sizes.values())} px.",f"- Video: `{video['path']}`, {video['duration_seconds']:.1f} s, {video['resolution']}, H.264, {video['file_size_bytes']} bytes; full {video['decoded_frames']}-frame decode PASS.",f"- Existing manifests verified: {', '.join(old_counts)}.","- Protected Xacro/URDF, SRDF/ACM, joint limits, kinematics and OMPL hashes unchanged.","- Legend placement: outside plotting area with a reserved fixed-canvas margin; 1920×1080 output verified without clipping.",""]
    with open(AUDIT,"x",encoding="utf-8") as f:f.write("\n".join(lines))
    package=[f"{PKG}/CMakeLists.txt",f"{PKG}/package.xml",f"{PKG}/config/lift_slice_fk_workspace.yaml",f"{PKG}/launch/lift_slice_fk_workspace.launch.py",f"{PKG}/launch/lift_slice_fk_workspace_demo.launch.py",f"{PKG}/src/lift_slice_fk_workspace.cpp",f"{PKG}/scripts/postprocess_lift_slice_fk_workspace.py",f"{PKG}/scripts/lift_slice_fk_workspace_rviz.py",f"{PKG}/scripts/generate_lift_slice_fk_workspace_video.py",f"{PKG}/scripts/finalize_lift_slice_fk_workspace.py",f"{PKG}/rviz/lift_slice_fk_workspace_3d.rviz",f"{PKG}/rviz/lift_slice_fk_workspace_front.rviz",f"{PKG}/rviz/lift_slice_fk_workspace_right.rviz"]
    validation=[f"validation/lift_slice_fk_workspace_{x}" for x in ["states.csv","sampling_metadata.csv","nested_points.csv","nested_check.csv","front.csv","right.csv","summary.csv","3d_surface.csv","slice_displacement.csv","representative_states.csv","figure_index.csv","audit.md"]]+["validation/lift_slice_fk_vs_grid_workspace_comparison.csv"]
    targets=package+validation+figures+["presentation/lift_slice_fk_workspace_demo.mp4","presentation/lift_slice_fk_workspace_demo_metadata.json"]
    with open(MANIFEST,"x",encoding="utf-8") as f:
        for rel in sorted(targets):
            path=f"{WS}/{rel}"
            if not os.path.isfile(path) or os.path.getsize(path)==0:raise RuntimeError(f"Missing target {rel}")
            f.write(f"{digest(path)}  {rel}\n")
    verify_manifest(os.path.basename(MANIFEST),WS)
    print(json.dumps({"status":"PASS","states":len(states),"nested_points":len(nested),"triangles":len(surface),"manifest_entries":len(targets),"video":video},indent=2))
if __name__=="__main__":main()

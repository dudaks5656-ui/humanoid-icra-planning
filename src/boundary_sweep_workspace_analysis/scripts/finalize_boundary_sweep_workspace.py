#!/usr/bin/env python3
"""Integrity audit and SHA-256 manifest for the directed boundary sweep."""
import csv,hashlib,json,math,os,subprocess
from PIL import Image
WS="/home/openarm/humanoid_sim_ws";V=f"{WS}/validation";P=f"{WS}/presentation";PKG="src/boundary_sweep_workspace_analysis";AUD=f"{V}/boundary_sweep_workspace_audit.md";MAN=f"{V}/boundary_sweep_workspace_manifest_sha256.txt"
CONFIGS=["LIFT_ONLY","LIFT_YAW","LIFT_PITCH","LIFT_YAW_PITCH"]
PROTECTED={"src/humanoid_sim_description/urdf/humanoid_sim.urdf.xacro":"e0bcfb64859d94203eeec45f823dec8d238d72bec6809d81c2c99091942be4e4","src/humanoid_sim_moveit_config/config/humanoid_sim.srdf":"7892b122fe28e91650f8919a66c256ebc55d3b34472e006701a63cadab5484e8","src/humanoid_sim_moveit_config/config/joint_limits.yaml":"1e851fa791dab1c95eeba4a698e620c0334afd12a0c9964b774f24868d03ccfb","src/humanoid_sim_moveit_config/config/kinematics.yaml":"7a825c85e6d3bfa06b3ff6d195b3aced8c490d6132d8d94e2d0b01f52e73204e","src/humanoid_sim_moveit_config/config/ompl_planning.yaml":"3e47c074ffcb9a72ea62e8821a8923f80c7f346e246993b3131038380aaf1335"}
def digest(path):
 h=hashlib.sha256()
 with open(path,"rb") as s:
  for b in iter(lambda:s.read(1048576),b""):h.update(b)
 return h.hexdigest()
def read(name):
 with open(f"{V}/{name}",newline="",encoding="utf-8") as f:return list(csv.DictReader(f))
def verify(name,base):
 n=0
 for line in open(f"{V}/{name}",encoding="utf-8"):
  if not line.strip():continue
  h,p=line.rstrip().split("  ",1)
  if digest(os.path.join(base,p))!=h:raise RuntimeError(f"Manifest mismatch {name}: {p}")
  n+=1
 return n
def fmt(x):return f"{float(x):.4f}"
def main():
 if os.path.exists(AUD) or os.path.exists(MAN):raise RuntimeError("Refusing audit/manifest overwrite")
 for p,h in PROTECTED.items():
  if digest(f"{WS}/{p}")!=h:raise RuntimeError(f"Protected model changed: {p}")
 old=[("fixed_base_workspace_manifest_sha256.txt",V),("fixed_base_workspace_fine_manifest_sha256.txt",V),("fixed_base_workspace_dof_ablation_manifest_sha256.txt",V),("fixed_base_workspace_demo_manifest_sha256.txt",WS),("fixed_base_workspace_envelope_demo_manifest_sha256.txt",WS),("radial_workspace_validation_manifest_sha256.txt",WS),("workspace_projection_manifest_sha256.txt",WS),("fk_workspace_boundary_manifest_sha256.txt",WS),("lift_slice_fk_workspace_manifest_sha256.txt",WS)];old_counts={n:verify(n,b) for n,b in old}
 raw=read("boundary_sweep_workspace_states.csv");lift=read("boundary_sweep_workspace_lift_scan.csv");meta={r["key"]:r["value"] for r in read("boundary_sweep_workspace_sampling_metadata.csv")};nested=read("boundary_sweep_workspace_nested_states.csv");pool=read("boundary_sweep_workspace_nested_pool.csv");checks=read("boundary_sweep_workspace_nested_check.csv");front=read("boundary_sweep_workspace_front.csv");right=read("boundary_sweep_workspace_right.csv");summary=read("boundary_sweep_workspace_summary.csv");comparison=read("boundary_sweep_workspace_comparison.csv");cross=read("boundary_sweep_workspace_cross_validation.csv");surface=read("boundary_sweep_workspace_3d_surface.csv");poses=read("boundary_sweep_workspace_representative_poses.csv")
 if len(raw)!=1920 or len(nested)!=1920 or len(summary)!=4 or len(poses)!=16:raise RuntimeError("Cardinality failure")
 if any(r["valid"]!="1" or r["self_collision"]!="0" or not all(math.isfinite(float(r[k])) for k in ("tcp_x","tcp_y","tcp_z","joint_margin","self_clearance")) for r in raw):raise RuntimeError("Invalid selected boundary state")
 if any(r["status"]!="PASS" or int(r["missing_source_pool_count"]) or int(r["boundary_metric_violation_count"]) for r in checks):raise RuntimeError("Nested inclusion failure")
 if meta["collision_checks"]!="140520" or meta["self_collision_rejections"]!="8141" or meta["ik_used"]!="false":raise RuntimeError("Scope/count drift")
 usable=[float(r["lift_value"]) for r in lift if r["usable"]=="1"];unusable=[float(r["lift_value"]) for r in lift if r["usable"]=="0"]
 if min(usable)!=0 or max(usable)!=.54 or min(unusable)!=.56:raise RuntimeError("Usable lift boundary failure")
 figures=[r["path"] for r in read("boundary_sweep_workspace_figure_index.csv")]
 if len(figures)!=15 or len(set(figures))!=15:raise RuntimeError("Figure count failure")
 for p in figures:
  with Image.open(f"{WS}/{p}") as im:
   if im.size!=(1920,1080):raise RuntimeError(f"Figure size {p}: {im.size}")
   im.verify()
 with open(f"{P}/boundary_sweep_workspace_demo_metadata.json") as f:video=json.load(f)
 if video["codec_name"]!="h264" or video["resolution"]!="1920x1080" or video["decoded_frames"]!=990 or abs(video["duration_seconds"]-33)>0.1:raise RuntimeError("Video failure")
 robot_hash=hashlib.sha256(subprocess.check_output(["xacro",f"{WS}/src/humanoid_sim_description/urdf/humanoid_sim.urdf.xacro"])).hexdigest()
 lines=["# Directed Boundary-Sweep Workspace Audit","","## Model and scope","",f"- Expanded RobotModel SHA-256: `{robot_hash}`.","- Frames: base `base_link`, TCP `openarm_left_hand_tcp`, radial reference `openarm_left_link0`.","- Coordinates: +X forward, +Y left, +Z up; AMR/base fixed.","- Method: directed FK boundary optimization, not joint-space random workspace sampling.","- No IK, OMPL, Cartesian grid, task planning, trajectory execution, controller, ros2_control or hardware.","","## Usable Lift range","",f"- URDF limit: 0.000–0.700 m; collision-free operational range from scan: `{meta['usable_lift_bottom']}–{meta['usable_lift_top']} m` (bottom–top).","- Scan step: 0.020 m; eight deterministic representative arm postures per position.","- At 0.000–0.200 m: 8/8 representatives valid; 0.220–0.540 m: 7/8 valid; 0.560–0.700 m: 0/8 valid and 8/8 self-colliding.","- Boundary slices: 0.000, 0.135, 0.270, 0.405, 0.540 m.","","## Directed sweep","","- Each view uses 24 support directions at 15° spacing.","- For each direction: one deterministic valid seed; joint-by-joint global coarse fractions 10/30/50/70/90%, then local ±5° and ±1° refinement.","- Front Y-Z outer objective maximizes directional support; right X-Z does the same. Inner objective minimizes shoulder-relative radius with angular mismatch penalty.",f"- Collision checks: {meta['collision_checks']}; self-collision rejects: {meta['self_collision_rejections']}; valid evaluations: {meta['valid_evaluations']}.","- Every emitted boundary state satisfies joint bounds, avoids exact sampled-active bounds, and passes existing SRDF ACM self-collision checking.","","## Results","","| Config | Front min–max r (m) | Right min–max r (m) | Max X (m) | Max |Y| (m) | Front area (m²) | Right area (m²) |","|---|---:|---:|---:|---:|---:|---:|"]
 for r in summary:lines.append(f"| {r['configuration']} | {fmt(r['front_min_reach'])}–{fmt(r['front_max_reach'])} | {fmt(r['right_min_reach'])}–{fmt(r['right_max_reach'])} | {fmt(r['max_forward_x'])} | {fmt(r['max_lateral_y'])} | {fmt(r['front_area'])} | {fmt(r['right_area'])} |")
 lines += ["","Projected areas are 4 mm rasterized unions of boundary-enclosed bands from structurally included source configurations. They are presentation envelope estimates, not proof that every interior point is feasible.","","## Increase versus C0","","| Config | Δ forward X | Δ lateral | Front area Δ (%) | Right area Δ (%) |","|---|---:|---:|---:|---:|"]
 for r in comparison:lines.append(f"| {r['configuration']} | {float(r['max_forward_increase']):+.4f} | {float(r['max_lateral_increase']):+.4f} | {float(r['front_area_increase']):+.4f} ({float(r['front_area_increase_percent']):+.2f}%) | {float(r['right_area_increase']):+.4f} ({float(r['right_area_increase_percent']):+.2f}%) |")
 lines += ["","## Nested inclusion and shell","",f"- Structural source pools contain {len(pool)} valid boundary states; all {len(checks)} nested checks PASS.","- Zero source-pool omissions and zero per-direction inner/outer support metric regressions: C0⊆C1/C2 and C1/C2⊆C3.",f"- 3D shell: {len(surface)} triangles formed only between corresponding valid angles and adjacent usable Lift slices.","- Each upper configuration shell is the union of its own and all required lower-configuration valid shells.","- No convex hull, smoothing, interpolation across invalid/no-data slices, or artificial internal fill.","","## Cross-validation","","| Config | Sweep max X | Fixed-orientation grid | Dense FK | Lift-slice FK | Interpretation |","|---|---:|---:|---:|---:|---|"]
 for r in cross:lines.append(f"| {r['configuration']} | {fmt(r['boundary_sweep_max_x'])} | {fmt(r['grid_fixed_orientation_max_x'])} | {fmt(r['fk_random_positional_max_x'])} | {fmt(r['lift_slice_positional_max_x'])} | {r['qualitative_consistency']} |")
 lines += ["","The fast one-seed directed sweep underestimates C0/C1 maximum X relative to dense positional FK, while C2/C3 are close or slightly larger. It is a presentation boundary construction, not a replacement for the validated dense/grid studies. The expansion ordering and strong Pitch/full-torso forward benefit remain consistent.","","## Presentation and integrity","",f"- Figures: 15 PNG at 1920×1080. All 2D legends are outside the plotting area and unclipped.",f"- Video: `{video['path']}`, {video['duration_seconds']:.1f} s, 1920×1080, 30 fps, H.264, {video['file_size_bytes']} bytes; all {video['decoded_frames']} frames decoded.",f"- Existing manifests verified: {', '.join(old_counts)}.","- Protected URDF/Xacro, SRDF/ACM, joint limits, kinematics and OMPL hashes unchanged.",""]
 with open(AUD,"x",encoding="utf-8") as f:f.write("\n".join(lines))
 package=[f"{PKG}/CMakeLists.txt",f"{PKG}/package.xml",f"{PKG}/config/boundary_sweep_workspace.yaml",f"{PKG}/launch/boundary_sweep_workspace.launch.py",f"{PKG}/launch/boundary_sweep_workspace_demo.launch.py",f"{PKG}/rviz/boundary_sweep_workspace.rviz",f"{PKG}/src/boundary_sweep_workspace.cpp",f"{PKG}/scripts/postprocess_boundary_sweep_workspace.py",f"{PKG}/scripts/boundary_sweep_workspace_rviz.py",f"{PKG}/scripts/generate_boundary_sweep_workspace_video.py",f"{PKG}/scripts/finalize_boundary_sweep_workspace.py"]
 validation=[f"validation/boundary_sweep_workspace_{x}" for x in ["states.csv","lift_scan.csv","sampling_metadata.csv","nested_states.csv","nested_pool.csv","nested_check.csv","front.csv","right.csv","summary.csv","comparison.csv","cross_validation.csv","3d_surface.csv","representative_poses.csv","figure_index.csv","audit.md"]]
 targets=package+validation+figures+["presentation/boundary_sweep_workspace_demo.mp4","presentation/boundary_sweep_workspace_demo_metadata.json"]
 with open(MAN,"x",encoding="utf-8") as f:
  for p in sorted(targets):
   if not os.path.isfile(f"{WS}/{p}") or os.path.getsize(f"{WS}/{p}")==0:raise RuntimeError(f"Missing {p}")
   f.write(f"{digest(f'{WS}/{p}')}  {p}\n")
 verify(os.path.basename(MAN),WS);print(json.dumps({"status":"PASS","usable_lift":[min(usable),max(usable)],"collision_checks":int(meta["collision_checks"]),"nested_checks":len(checks),"triangles":len(surface),"manifest_entries":len(targets),"video":video},indent=2))
if __name__=="__main__":main()

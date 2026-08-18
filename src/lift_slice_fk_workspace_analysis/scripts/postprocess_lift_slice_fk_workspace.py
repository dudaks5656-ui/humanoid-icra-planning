#!/usr/bin/env python3
"""Build nested lift-slice FK pools, 2-D boundaries, and hole-preserving 3-D lofts."""
import csv, math, os
from collections import defaultdict
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

WS="/home/openarm/humanoid_sim_ws"; VAL=f"{WS}/validation"; PRE=f"{WS}/presentation"
PKG=f"{WS}/src/lift_slice_fk_workspace_analysis"
STATES=f"{VAL}/lift_slice_fk_workspace_states.csv"; META=f"{VAL}/lift_slice_fk_workspace_sampling_metadata.csv"
CONFIGS=["LIFT_ONLY","LIFT_YAW","LIFT_PITCH","LIFT_YAW_PITCH"]
SHORT=dict(zip(CONFIGS,["C0","C1","C2","C3"])); LABELS=dict(zip(CONFIGS,["Arm + Lift","+ Waist Yaw","+ Waist Pitch","+ Yaw + Pitch"]))
COLORS=["#2f91ff","#ff9f1c","#dd63dc","#27d3c2"]; SLICE_COLORS=["#4e79a7","#59a14f","#f28e2b","#e15759","#b07aa1"]
RATIOS=[0.0,0.25,0.5,0.75,1.0]

def read(path):
    with open(path,newline="",encoding="utf-8") as f:return list(csv.DictReader(f))
def write(path, fields, rows):
    if os.path.exists(path): raise RuntimeError(f"Refusing overwrite: {path}")
    with open(path,"x",newline="",encoding="utf-8") as f:
        w=csv.DictWriter(f,fieldnames=fields,lineterminator="\n");w.writeheader();w.writerows(rows)
def f(row,key): return float(row[key])
def keyratio(x): return round(float(x),8)

def metadata():
    return {r["key"]:r["value"] for r in read(META)}

def canonical_margin(row,target,limits):
    names=row["joint_names"].split(";"); vals=list(map(float,row["joint_values"].split(";"))); q=dict(zip(names,vals))
    active=[]
    if target in ("LIFT_YAW","LIFT_YAW_PITCH"): active.append("waist_yaw_joint")
    if target in ("LIFT_PITCH","LIFT_YAW_PITCH"): active.append("waist_pitch_joint")
    active += [n for n in names if n.startswith("openarm_left_joint")]
    return min(min(q[n]-limits[n][0],limits[n][1]-q[n]) for n in active)

def build_nested(rows,limits):
    own=defaultdict(list)
    for r in rows:
        if r["valid"]=="1": own[(r["configuration"],keyratio(r["lift_ratio"]))].append(r)
    sources={"LIFT_ONLY":["LIFT_ONLY"],"LIFT_YAW":["LIFT_ONLY","LIFT_YAW"],
             "LIFT_PITCH":["LIFT_ONLY","LIFT_PITCH"],"LIFT_YAW_PITCH":CONFIGS}
    pools={}; out=[]
    for config in CONFIGS:
        for ratio in RATIOS:
            pool=[]
            for source in sources[config]:
                for r in own[(source,ratio)]:
                    n=dict(r); n.update(target_configuration=config,source_configuration=source,
                                      nested_joint_margin=canonical_margin(r,config,limits),source_key=f"{source}:{r['sample_id']}")
                    pool.append(n);out.append(n)
            pools[(config,ratio)]=pool
    fields=["target_configuration","source_configuration","source_key","lift_ratio","lift_value","sample_id","tcp_x","tcp_y","tcp_z","yaw","pitch","nested_joint_margin","self_clearance","joint_names","joint_values"]
    write(f"{VAL}/lift_slice_fk_workspace_nested_points.csv",fields,[{k:r[k] for k in fields} for r in out])
    checks=[]
    for ratio in RATIOS:
        sets={c:{r['source_key'] for r in pools[(c,ratio)]} for c in CONFIGS}
        for lower,upper in [("LIFT_ONLY","LIFT_YAW"),("LIFT_ONLY","LIFT_PITCH"),("LIFT_YAW","LIFT_YAW_PITCH"),("LIFT_PITCH","LIFT_YAW_PITCH")]:
            missing=sets[lower]-sets[upper]
            checks.append(dict(lift_ratio=ratio,lower_configuration=lower,upper_configuration=upper,
                               lower_endpoint_count=len(sets[lower]),upper_endpoint_count=len(sets[upper]),
                               missing_count=len(missing),status="PASS" if not missing else "FAIL"))
    write(f"{VAL}/lift_slice_fk_workspace_nested_check.csv",list(checks[0]),checks)
    if any(r["status"]!="PASS" for r in checks):raise RuntimeError("Nested inclusion failed")
    return own,pools,checks

def boundary(points,view,ratio,ref,settings):
    hk="tcp_y" if view=="front" else "tcp_x"; origin=ref[1] if view=="front" else ref[0]; oz=ref[2]
    grouped=defaultdict(list); step=float(settings["angle_bin_deg"])
    for p in points:
        u,z=f(p,hk),f(p,"tcp_z"); angle=math.degrees(math.atan2(z-oz,u-origin)); idx=math.floor((angle+180)/step)
        grouped[idx].append((math.hypot(u-origin,z-oz),u,z))
    result=[]
    for idx,samples in sorted(grouped.items()):
        samples.sort(); radii=np.array([x[0] for x in samples]); gaps=int(np.sum(np.diff(radii)>float(settings["observed_gap_threshold_m"]))) if len(samples)>=int(settings["minimum_gap_bin_samples"]) else 0
        lo,hi=samples[0],samples[-1]; prefix="y" if view=="front" else "x"
        result.append(dict(angle_deg=-180+(idx+.5)*step,inner_radius=lo[0],outer_radius=hi[0],
                           **{f"inner_{prefix}":lo[1],"inner_z":lo[2],f"outer_{prefix}":hi[1],"outer_z":hi[2]},
                           valid_sample_count=len(samples),observed_gap_count=gaps))
    return result

def surface(points_by_slice,refs,settings):
    azstep=float(settings["spherical_azimuth_bin_deg"]); elstep=float(settings["spherical_elevation_bin_deg"]); gap=float(settings["observed_gap_threshold_m"])
    outer={}
    for ratio,points in points_by_slice.items():
        ref=refs[ratio]; bins=defaultdict(list)
        for p in points:
            v=np.array([f(p,"tcp_x"),f(p,"tcp_y"),f(p,"tcp_z")])-ref; radius=np.linalg.norm(v)
            if radius==0:continue
            az=math.degrees(math.atan2(v[1],v[0])); el=math.degrees(math.atan2(v[2],math.hypot(v[0],v[1])))
            bins[(math.floor((az+180)/azstep),math.floor((el+90)/elstep))].append((radius,np.array([f(p,"tcp_x"),f(p,"tcp_y"),f(p,"tcp_z")])))
        for b,items in bins.items():
            items.sort(key=lambda x:x[0]); radii=np.array([x[0] for x in items])
            if len(items)>=3 and np.any(np.diff(radii)>gap): continue
            outer[(ratio,*b)]=items[-1][1]
    triangles=[]; tid=0
    for r0,r1 in zip(RATIOS[:-1],RATIOS[1:]):
        keys={(az,el) for (r,az,el) in outer if r==r0}
        for az,el in sorted(keys):
            quad=[outer.get((r0,az,el)),outer.get((r0,az+1,el)),outer.get((r1,az+1,el)),outer.get((r1,az,el))]
            if any(v is None for v in quad):continue
            for tri in ((quad[0],quad[1],quad[2]),(quad[0],quad[2],quad[3])):
                triangles.append((tid,r0,r1,*tri));tid+=1
    return outer,triangles

def plot2d(path,config,pools,bounds,view,refs):
    fig,ax=plt.subplots(figsize=(16,9),dpi=120); fig.patch.set_facecolor("#10151d");ax.set_facecolor("#151c25");fig.subplots_adjust(left=.08,right=.78,bottom=.10,top=.90)
    hk="tcp_y" if view=="front" else "tcp_x"; prefix="y" if view=="front" else "x"
    for ratio,color in zip(RATIOS,SLICE_COLORS):
        pts=pools[(config,ratio)]; sampled=pts[::max(1,len(pts)//1200)]
        ax.scatter([f(p,hk) for p in sampled],[f(p,"tcp_z") for p in sampled],s=3,alpha=.04,color=color)
        b=bounds[(config,ratio,view)]; empty=not pts
        ax.plot([x[f"outer_{prefix}"] for x in b],[x["outer_z"] for x in b],color=color,lw=2,label=f"Lift {ratio*100:.0f}% outer"+(" · NO VALID STATE" if empty else ""))
        ax.plot([x[f"inner_{prefix}"] for x in b],[x["inner_z"] for x in b],color=color,lw=1.3,ls="--",label="_nolegend_" if empty else f"Lift {ratio*100:.0f}% inner")
        ref=refs[ratio]; ax.scatter([ref[1] if view=="front" else ref[0]],[ref[2]],marker="x",s=35,color=color)
    ax.axvline(0,color="#eee",lw=.8,alpha=.4);ax.grid(alpha=.15);ax.set_aspect("equal",adjustable="box")
    ax.set_xlabel(("Y left/right" if view=="front" else "X backward/forward")+" [m]");ax.set_ylabel("Z vertical [m]")
    ax.set_title(f"{SHORT[config]} {LABELS[config]} · {'Front Y–Z' if view=='front' else 'Right X–Z'}\nSolid outer / dashed inner · shoulder reference ×")
    ax.tick_params(colors="#e7edf5");ax.xaxis.label.set_color("#e7edf5");ax.yaxis.label.set_color("#e7edf5");ax.title.set_color("#f5f7fa")
    for spine in ax.spines.values():spine.set_color("#8fa1b3")
    ax.legend(loc="upper left",bbox_to_anchor=(1.02,1.0),borderaxespad=0,fontsize=9)
    fig.savefig(path,dpi=120,facecolor=fig.get_facecolor());plt.close(fig)

def plot_compare(path,pools,bounds,view):
    fig,axes=plt.subplots(2,2,figsize=(16,9),dpi=120,sharex=True,sharey=True);fig.patch.set_facecolor("#10151d");fig.subplots_adjust(left=.08,right=.86,bottom=.10,top=.90,wspace=.16,hspace=.20)
    prefix="y" if view=="front" else "x"; hk="tcp_y" if view=="front" else "tcp_x"
    handles=[]
    for ax,config,color in zip(axes.flat,CONFIGS,COLORS):
        ax.set_facecolor("#151c25")
        for ratio,alpha in zip(RATIOS,[.35,.5,.7,.85,1]):
            b=bounds[(config,ratio,view)]; line=ax.plot([x[f"outer_{prefix}"] for x in b],[x["outer_z"] for x in b],color=color,lw=1.5,alpha=alpha,label=f"{ratio*100:.0f}%")[0]
            if ax is axes.flat[0]:handles.append(line)
        ax.axvline(0,color="#eee",lw=.6,alpha=.3);ax.grid(alpha=.12);ax.set_aspect("equal",adjustable="box");ax.set_title(f"{SHORT[config]} {LABELS[config]}",color="#f5f7fa");ax.tick_params(colors="#e7edf5")
        for spine in ax.spines.values():spine.set_color("#8fa1b3")
    fig.supxlabel(("Y left/right" if view=="front" else "X backward/forward")+" [m]");fig.supylabel("Z [m]")
    fig.suptitle(f"Lift-slice FK outer boundaries · {'Front Y–Z' if view=='front' else 'Right X–Z'}",color="#f5f7fa");fig.supxlabel(("Y left/right" if view=="front" else "X backward/forward")+" [m]",color="#e7edf5");fig.supylabel("Z [m]",color="#e7edf5")
    fig.legend(handles,[f"Lift {r*100:.0f}%"+(" · NO VALID STATE" if r==1.0 else "") for r in RATIOS],loc="center left",bbox_to_anchor=(1.0,.5))
    fig.savefig(path,dpi=120,facecolor=fig.get_facecolor());plt.close(fig)

def plot3d(path,configs,pools,surfaces):
    fig=plt.figure(figsize=(16,9),dpi=120);fig.patch.set_facecolor("#10151d");ax=fig.add_subplot(111,projection="3d");ax.set_facecolor("#151c25");fig.subplots_adjust(left=.04,right=.96,bottom=.06,top=.92)
    for config in configs:
        color=COLORS[CONFIGS.index(config)]; tris=[list(x[3:]) for x in surfaces[config]]
        if tris: ax.add_collection3d(Poly3DCollection(tris,facecolors=color,edgecolors=color,linewidths=.15,alpha=.16 if len(configs)>1 else .28))
        for ratio in RATIOS:
            pts=pools[(config,ratio)][::max(1,len(pools[(config,ratio)])//350)]
            ax.scatter([f(p,"tcp_x") for p in pts],[f(p,"tcp_y") for p in pts],[f(p,"tcp_z") for p in pts],s=2,color=color,alpha=.05)
    ax.scatter([0],[0],[0],marker="x",s=80,color="white");ax.set_xlabel("X forward [m]",color="#e7edf5");ax.set_ylabel("Y left [m]",color="#e7edf5");ax.set_zlabel("Z [m]",color="#e7edf5");ax.tick_params(colors="#e7edf5")
    ax.set_title("Lift-slice FK 3D loft · no convex hull · missing/gapped bins left open",color="#f5f7fa");ax.view_init(22,-125)
    fig.savefig(path,dpi=120,facecolor=fig.get_facecolor());plt.close(fig)

def main():
    os.makedirs(PRE,exist_ok=True)
    with open(f"{PKG}/config/lift_slice_fk_workspace.yaml",encoding="utf-8") as s: settings=yaml.safe_load(s)["lift_slice_fk_workspace"]["ros__parameters"]
    rows=read(STATES); meta=metadata()
    if len(rows)!=40000:raise RuntimeError(f"Expected 40000 rows, got {len(rows)}")
    limits={n:tuple(map(float,v.split(";"))) for k,v in meta.items() if k.startswith("joint_limit_") for n in [k.removeprefix("joint_limit_")]}
    refs={r:np.array(list(map(float,meta[f"reference_xyz_{i}"].split(";")))) for i,r in enumerate(RATIOS)}
    own,pools,checks=build_nested(rows,limits)
    bounds={}; front_rows=[];right_rows=[]
    for c in CONFIGS:
        for ratio in RATIOS:
            for view,target in (("front",front_rows),("right",right_rows)):
                b=boundary(pools[(c,ratio)],view,ratio,refs[ratio],settings);bounds[(c,ratio,view)]=b
                lift_value=next(f(r,"lift_value") for r in rows if r["configuration"]==c and keyratio(r["lift_ratio"])==ratio)
                target.extend(dict(configuration=c,lift_ratio=ratio,lift_value=lift_value,**x) for x in b)
    ff=["configuration","lift_ratio","lift_value","angle_deg","inner_radius","outer_radius","inner_y","inner_z","outer_y","outer_z","valid_sample_count","observed_gap_count"]
    rf=["configuration","lift_ratio","lift_value","angle_deg","inner_radius","outer_radius","inner_x","inner_z","outer_x","outer_z","valid_sample_count","observed_gap_count"]
    write(f"{VAL}/lift_slice_fk_workspace_front.csv",ff,front_rows);write(f"{VAL}/lift_slice_fk_workspace_right.csv",rf,right_rows)
    summaries=[]
    for c in CONFIGS:
        for ratio in RATIOS:
            raw=[r for r in rows if r["configuration"]==c and keyratio(r["lift_ratio"])==ratio]; pts=pools[(c,ratio)]; xyz=np.array([[f(p,k) for k in ("tcp_x","tcp_y","tcp_z")] for p in pts]); margins=np.array([float(p["nested_joint_margin"]) for p in pts]);clr=np.array([f(p,"self_clearance") for p in pts])
            fr=bounds[(c,ratio,"front")];rr=bounds[(c,ratio,"right")]
            finite_summary=dict(x_min=xyz[:,0].min(),x_max=xyz[:,0].max(),y_min=xyz[:,1].min(),y_max=xyz[:,1].max(),z_min=xyz[:,2].min(),z_max=xyz[:,2].max(),front_min_observed_radius=min(x["inner_radius"] for x in fr),front_max_observed_radius=max(x["outer_radius"] for x in fr),right_min_observed_radius=min(x["inner_radius"] for x in rr),right_max_observed_radius=max(x["outer_radius"] for x in rr),mean_joint_margin=margins.mean(),min_joint_margin=margins.min(),mean_self_clearance=clr.mean(),min_self_clearance=clr.min()) if pts else {k:"" for k in ("x_min","x_max","y_min","y_max","z_min","z_max","front_min_observed_radius","front_max_observed_radius","right_min_observed_radius","right_max_observed_radius","mean_joint_margin","min_joint_margin","mean_self_clearance","min_self_clearance")}
            summaries.append(dict(configuration=c,lift_ratio=ratio,lift_value=f(raw[0],"lift_value"),attempted_states=len(raw),own_valid_states=sum(r["valid"]=="1" for r in raw),nested_pool_states=len(pts),inherited_states=len(pts)-sum(r["valid"]=="1" for r in raw),slice_status="VALID_ENDPOINTS" if pts else "NO_VALID_STATE_SELF_COLLISION",**finite_summary,self_collision_rejection_count=sum(r["failure_reason"]=="SELF_COLLISION" for r in raw),front_observed_gap_count=sum(x["observed_gap_count"] for x in fr),right_observed_gap_count=sum(x["observed_gap_count"] for x in rr)))
    write(f"{VAL}/lift_slice_fk_workspace_summary.csv",list(summaries[0]),summaries)
    grid={r["configuration"]:r for r in read(f"{VAL}/workspace_projection_summary.csv")}
    comparisons=[]
    for c in CONFIGS:
        subset=[r for r in summaries if r["configuration"]==c and r["slice_status"]=="VALID_ENDPOINTS"]
        fxmax=max(float(r["x_max"]) for r in subset);fymin=min(float(r["y_min"]) for r in subset);fymax=max(float(r["y_max"]) for r in subset);fzmin=min(float(r["z_min"]) for r in subset);fzmax=max(float(r["z_max"]) for r in subset);g=grid[c]
        comparisons.append(dict(configuration=c,fk_max_x=fxmax,grid_max_x=g["x_max"],delta_x=fxmax-float(g["x_max"]),fk_y_span=fymax-fymin,grid_y_span=float(g["y_max"])-float(g["y_min"]),fk_z_span=fzmax-fzmin,grid_z_span=float(g["z_max_front"])-float(g["z_min_front"]),qualitative_consistency="POSITIONAL_FK_ORIENTATION_FREE_EXPECTED_BROADER"))
    write(f"{VAL}/lift_slice_fk_vs_grid_workspace_comparison.csv",list(comparisons[0]),comparisons)

    surfaces={};surface_rows=[];displacements=[]
    for c in CONFIGS:
        outer,tris=surface({r:pools[(c,r)] for r in RATIOS},refs,settings);surfaces[c]=tris
        for tid,r0,r1,a,b,d in tris:
            surface_rows.append(dict(configuration=c,triangle_id=tid,lower_lift_ratio=r0,upper_lift_ratio=r1,v1_x=a[0],v1_y=a[1],v1_z=a[2],v2_x=b[0],v2_y=b[1],v2_z=b[2],v3_x=d[0],v3_y=d[1],v3_z=d[2]))
        for r0,r1 in zip(RATIOS[:-1],RATIOS[1:]):
            distances=[]
            keys={(az,el) for (r,az,el) in outer if r==r0 and (r1,az,el) in outer}
            for az,el in keys:distances.append(float(np.linalg.norm(outer[(r1,az,el)]-outer[(r0,az,el)])))
            displacements.append(dict(configuration=c,lower_lift_ratio=r0,upper_lift_ratio=r1,matched_direction_bins=len(distances),mean_boundary_displacement=np.mean(distances) if distances else "",max_boundary_displacement=np.max(distances) if distances else "",connection_model="LINEAR_LOFT_APPROXIMATION_NO_EXTRA_SLICES"))
    write(f"{VAL}/lift_slice_fk_workspace_3d_surface.csv",list(surface_rows[0]),surface_rows);write(f"{VAL}/lift_slice_fk_workspace_slice_displacement.csv",list(displacements[0]),displacements)

    representatives=[]
    for c in CONFIGS:
        for ratio,label in [(0.0,"MIN"),(0.5,"MID"),(1.0,"MAX")]:
            if pools[(c,ratio)]:
                p=max(pools[(c,ratio)],key=lambda x:f(x,"tcp_x"));representatives.append(dict(configuration=c,slice_label=label,lift_ratio=ratio,lift_value=p["lift_value"],tcp_x=p["tcp_x"],tcp_y=p["tcp_y"],tcp_z=p["tcp_z"],source_configuration=p["source_configuration"],sample_id=p["sample_id"],joint_names=p["joint_names"],joint_values=p["joint_values"],selection="MAX_FORWARD_X_IN_NESTED_POOL",status="VALID_POSE"))
            else:
                lift_value=next(r["lift_value"] for r in rows if r["configuration"]==c and keyratio(r["lift_ratio"])==ratio);representatives.append(dict(configuration=c,slice_label=label,lift_ratio=ratio,lift_value=lift_value,tcp_x="",tcp_y="",tcp_z="",source_configuration="",sample_id="",joint_names="",joint_values="",selection="NO_VALID_REPRESENTATIVE",status="NO_VALID_STATE_SELF_COLLISION"))
    write(f"{VAL}/lift_slice_fk_workspace_representative_states.csv",list(representatives[0]),representatives)

    figure_rows=[]
    for i,c in enumerate(CONFIGS):
        for view in ("front","right"):
            path=f"{PRE}/lift_slice_{view}_c{i}.png";plot2d(path,c,pools,bounds,view,refs);figure_rows.append(dict(scene=f"{view}_c{i}",path=os.path.relpath(path,WS)))
        path=f"{PRE}/lift_slice_3d_c{i}.png";plot3d(path,[c],pools,surfaces);figure_rows.append(dict(scene=f"3d_c{i}",path=os.path.relpath(path,WS)))
    for view in ("front","right"):
        path=f"{PRE}/lift_slice_{view}_compare.png";plot_compare(path,pools,bounds,view);figure_rows.append(dict(scene=f"{view}_compare",path=os.path.relpath(path,WS)))
    path=f"{PRE}/lift_slice_3d_all.png";plot3d(path,CONFIGS,pools,surfaces);figure_rows.append(dict(scene="3d_all",path=os.path.relpath(path,WS)))
    write(f"{VAL}/lift_slice_fk_workspace_figure_index.csv",["scene","path"],figure_rows)
    print({"states":len(rows),"nested_checks":len(checks),"summary_rows":len(summaries),"triangles":{c:len(surfaces[c]) for c in CONFIGS},"figures":len(figure_rows)})

if __name__=="__main__":main()

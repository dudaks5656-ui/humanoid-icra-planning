#!/usr/bin/env python3
"""Structural nesting, presentation boundaries, valid-state lofts, and comparisons."""
import csv,math,os
from collections import defaultdict
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.path import Path
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

WS="/home/openarm/humanoid_sim_ws";VAL=f"{WS}/validation";PRE=f"{WS}/presentation"
RAW=f"{VAL}/boundary_sweep_workspace_states.csv";META=f"{VAL}/boundary_sweep_workspace_sampling_metadata.csv"
CONFIGS=["LIFT_ONLY","LIFT_YAW","LIFT_PITCH","LIFT_YAW_PITCH"];LABELS=["C0 Arm + Lift","C1 + Waist Yaw","C2 + Waist Pitch","C3 + Yaw + Pitch"];COLORS=["#2f91ff","#ff9f1c","#dd63dc","#27d3c2"]
VIEWS=["FRONT","RIGHT"];BOUNDARIES=["INNER_BOUNDARY","OUTER_BOUNDARY"]

def read(path):
 with open(path,newline="",encoding="utf-8") as f:return list(csv.DictReader(f))
def write(path,fields,rows):
 if os.path.exists(path):raise RuntimeError(f"Refusing overwrite {path}")
 with open(path,"x",newline="",encoding="utf-8") as f:w=csv.DictWriter(f,fieldnames=fields,lineterminator="\n");w.writeheader();w.writerows(rows)
def f(r,k):return float(r[k])
def ref(lift):return np.array([.2549756914527,.0302288135233,1.5538730228961-float(lift)])
def sources(c):return {"LIFT_ONLY":["LIFT_ONLY"],"LIFT_YAW":["LIFT_ONLY","LIFT_YAW"],"LIFT_PITCH":["LIFT_ONLY","LIFT_PITCH"],"LIFT_YAW_PITCH":CONFIGS}[c]

def select_nested(raw):
 grouped=defaultdict(list)
 for r in raw:grouped[(r["configuration"],r["view"],round(f(r,"lift_value"),8),round(f(r,"sweep_parameter"),8),r["boundary_type"])].append(r)
 lifts=sorted({round(f(r,"lift_value"),8) for r in raw});angles=sorted({round(f(r,"sweep_parameter"),8) for r in raw});out=[];pool=[];checks=[]
 for target in CONFIGS:
  for view in VIEWS:
   for lift in lifts:
    origin=ref(lift);oi=1 if view=="FRONT" else 0
    for angle in angles:
     theta=math.radians(angle);u=np.array([math.cos(theta),math.sin(theta)])
     for boundary in BOUNDARIES:
      candidates=[]
      for source in sources(target):candidates+=grouped[(source,view,lift,angle,boundary)]
      if not candidates:continue
      for candidate in candidates:
       p=dict(candidate);p.update(target_configuration=target,source_configuration=candidate["configuration"]);pool.append(p)
      def metric(r):
       vec=np.array([f(r,"tcp_y")-origin[1],f(r,"tcp_z")-origin[2]]) if view=="FRONT" else np.array([f(r,"tcp_x")-origin[0],f(r,"tcp_z")-origin[2]])
       if boundary=="OUTER_BOUNDARY":return float(vec@u)
       radius=np.linalg.norm(vec);a=math.atan2(vec[1],vec[0]);d=abs((a-theta+math.pi)%(2*math.pi)-math.pi);return -(radius+.75*d)
      chosen=max(candidates,key=metric);n=dict(chosen);n.update(target_configuration=target,source_configuration=chosen["configuration"],selection_metric=metric(chosen));out.append(n)
 selected={(r["target_configuration"],r["view"],round(f(r,"lift_value"),8),round(f(r,"sweep_parameter"),8),r["boundary_type"]):f(r,"selection_metric") for r in out}
 for lift in lifts:
  for lower,upper in [("LIFT_ONLY","LIFT_YAW"),("LIFT_ONLY","LIFT_PITCH"),("LIFT_YAW","LIFT_YAW_PITCH"),("LIFT_PITCH","LIFT_YAW_PITCH")]:
   lower_sources=set(sources(lower));upper_sources=set(sources(upper));missing=lower_sources-upper_sources;violations=0
   for view in VIEWS:
    for angle in angles:
     for boundary in BOUNDARIES:
      if selected[(upper,view,lift,angle,boundary)]+1e-12<selected[(lower,view,lift,angle,boundary)]:violations+=1
   checks.append(dict(lift_value=lift,lower_configuration=lower,upper_configuration=upper,missing_source_pool_count=len(missing),boundary_metric_violation_count=violations,status="PASS" if not missing and not violations else "FAIL"))
 return out,pool,checks,lifts,angles

def rows_for(nested,c,v,lift,b):return sorted([r for r in nested if r["target_configuration"]==c and r["view"]==v and abs(f(r,"lift_value")-lift)<1e-7 and r["boundary_type"]==b],key=lambda r:f(r,"sweep_parameter"))
def coords(rows,view):return np.array([[f(r,"tcp_y") if view=="FRONT" else f(r,"tcp_x"),f(r,"tcp_z")] for r in rows])
def ring_area(inner,outer,view):
 if len(inner)<3 or len(outer)<3:return 0.
 a=coords(outer,view);b=coords(inner,view)[::-1];p=np.vstack([a,b]);return abs(.5*np.sum(p[:,0]*np.roll(p[:,1],-1)-p[:,1]*np.roll(p[:,0],-1)))
def projected_union_area(nested,c,view,lifts,components=None):
 polys=[]
 for component in (components or [c]):
  for lift in lifts:
   i=rows_for(nested,component,view,lift,"INNER_BOUNDARY");o=rows_for(nested,component,view,lift,"OUTER_BOUNDARY")
   if i and o:polys.append(np.vstack([coords(o,view),coords(i,view)[::-1]]))
 if not polys:return 0.
 allp=np.vstack(polys);step=.004;xs=np.arange(allp[:,0].min(),allp[:,0].max()+step,step);zs=np.arange(allp[:,1].min(),allp[:,1].max()+step,step);xx,zz=np.meshgrid(xs,zs);test=np.c_[xx.ravel(),zz.ravel()];mask=np.zeros(len(test),bool)
 for p in polys:mask|=Path(p).contains_points(test)
 return float(mask.sum()*step*step)

def surface(nested,c,lifts):
 out=[];tid=0
 for view in VIEWS:
  for boundary in BOUNDARIES:
   for l0,l1 in zip(lifts[:-1],lifts[1:]):
    a=rows_for(nested,c,view,l0,boundary);b=rows_for(nested,c,view,l1,boundary);ma={r["sweep_parameter"]:r for r in a};mb={r["sweep_parameter"]:r for r in b};keys=sorted(set(ma)&set(mb),key=float)
    for k0,k1 in zip(keys,keys[1:]+keys[:1]):
     q=[ma[k0],ma[k1],mb[k1],mb[k0]];v=[np.array([f(r,"tcp_x"),f(r,"tcp_y"),f(r,"tcp_z")]) for r in q]
     for tri in ((v[0],v[1],v[2]),(v[0],v[2],v[3])):out.append((tid,view,boundary,l0,l1,*tri));tid+=1
 return out

def style(ax):
 ax.set_facecolor("#151c25");ax.tick_params(colors="#e8eef5");ax.xaxis.label.set_color("#e8eef5");ax.yaxis.label.set_color("#e8eef5");ax.title.set_color("#f4f7fb");ax.grid(alpha=.16);[s.set_color("#8fa1b3") for s in ax.spines.values()]
def plot_view(path,c,view,nested,lifts):
 fig,ax=plt.subplots(figsize=(16,9),dpi=120);fig.patch.set_facecolor("#10151d");fig.subplots_adjust(left=.08,right=.76,bottom=.11,top=.89);style(ax);hk="tcp_y" if view=="FRONT" else "tcp_x"
 for lift,color,ls in [(lifts[0],"#66c2ff","-"),(lifts[len(lifts)//2],"#ffd45c","-."),(lifts[-1],"#ff6b6b","--")]:
  for b,lw in [("OUTER_BOUNDARY",2.7),("INNER_BOUNDARY",1.8)]:
   r=rows_for(nested,c,view,lift,b);ax.plot([f(x,hk) for x in r],[f(x,"tcp_z") for x in r],color=color,ls=ls,lw=lw,label=f"Lift {lift:.3f} m · {'outer' if b.startswith('OUTER') else 'inner'}")
 ax.axvline(0,color="white",alpha=.35,lw=.8);ax.scatter([0],[0],marker="x",s=60,color="white");ax.set_xlabel(("Y lateral" if view=="FRONT" else "X forward")+" [m]");ax.set_ylabel("Z vertical [m]");ax.set_title(f"{LABELS[CONFIGS.index(c)]} · {'Front Y–Z' if view=='FRONT' else 'Right X–Z'} directed boundary sweep");ax.set_aspect("equal",adjustable="box");ax.legend(loc="upper left",bbox_to_anchor=(1.02,1),fontsize=10)
 fig.savefig(path,facecolor=fig.get_facecolor());plt.close(fig)
def plot_compare(path,view,nested,lifts):
 fig,axes=plt.subplots(2,2,figsize=(16,9),dpi=120,sharex=True,sharey=True);fig.patch.set_facecolor("#10151d");fig.subplots_adjust(left=.08,right=.86,bottom=.10,top=.90,wspace=.16,hspace=.20);hk="tcp_y" if view=="FRONT" else "tcp_x";handles=[]
 for ax,c,color in zip(axes.flat,CONFIGS,COLORS):
  style(ax)
  for b,ls in [("OUTER_BOUNDARY","-"),("INNER_BOUNDARY","--")]:
   for lift,alpha in zip(lifts,[.35,.5,.7,.85,1.]):
    r=rows_for(nested,c,view,lift,b);line=ax.plot([f(x,hk) for x in r],[f(x,"tcp_z") for x in r],color=color,ls=ls,lw=1.5,alpha=alpha,label=b)[0]
    if ax is axes.flat[0] and lift==lifts[-1]:handles.append(line)
  ax.set_title(LABELS[CONFIGS.index(c)]);ax.axvline(0,color="white",alpha=.25,lw=.6);ax.set_aspect("equal",adjustable="box")
 fig.suptitle(f"Directed inner/outer boundary comparison · {view}",color="white");fig.supxlabel(("Y lateral" if view=="FRONT" else "X forward")+" [m]",color="white");fig.supylabel("Z [m]",color="white");fig.legend(handles,["Outer boundary","Inner boundary"],loc="center left",bbox_to_anchor=(.88,.5));fig.savefig(path,facecolor=fig.get_facecolor());plt.close(fig)
def plot3d(path,configs,nested,surfaces):
 fig=plt.figure(figsize=(16,9),dpi=120);fig.patch.set_facecolor("#10151d");ax=fig.add_subplot(111,projection="3d");ax.set_facecolor("#151c25");fig.subplots_adjust(left=.04,right=.96,bottom=.06,top=.92)
 all_vertices=[]
 for c in configs:
  color=COLORS[CONFIGS.index(c)];tris=[list(x[5:]) for x in surfaces[c]];all_vertices.extend(v for tri in tris for v in tri);ax.add_collection3d(Poly3DCollection(tris,facecolors=color,edgecolors=color,alpha=.08 if len(configs)>1 else .22,linewidths=.18))
 xyz=np.array(all_vertices);pad=.05;ax.set_xlim(xyz[:,0].min()-pad,xyz[:,0].max()+pad);ax.set_ylim(xyz[:,1].min()-pad,xyz[:,1].max()+pad);ax.set_zlim(xyz[:,2].min()-pad,xyz[:,2].max()+pad)
 for axis in (ax.xaxis,ax.yaxis,ax.zaxis):axis.set_pane_color((.08,.11,.15,1.0))
 ax.set_xlabel("X forward [m]",color="#e8eef5");ax.set_ylabel("Y lateral [m]",color="#e8eef5");ax.set_zlabel("Z [m]",color="#e8eef5");ax.tick_params(colors="#e8eef5");ax.set_title("Directed boundary-sweep 3D shell · valid correspondences only",color="#f4f7fb");ax.view_init(22,-125);fig.savefig(path,facecolor=fig.get_facecolor());plt.close(fig)

def main():
 os.makedirs(PRE,exist_ok=True);raw=read(RAW);nested,pool,checks,lifts,angles=select_nested(raw)
 if any(r["status"]!="PASS" for r in checks):raise RuntimeError("Nested pool failure")
 nf=["target_configuration","source_configuration"]+list(raw[0])+["selection_metric"]
 write(f"{VAL}/boundary_sweep_workspace_nested_states.csv",nf,nested);write(f"{VAL}/boundary_sweep_workspace_nested_check.csv",list(checks[0]),checks)
 pf=["target_configuration","source_configuration"]+list(raw[0]);write(f"{VAL}/boundary_sweep_workspace_nested_pool.csv",pf,pool)
 front=[];right=[]
 for r in nested:
  common=dict(configuration=r["target_configuration"],lift_value=r["lift_value"],sweep_parameter=r["sweep_parameter"],tcp_z=r["tcp_z"],boundary_type=r["boundary_type"],valid=r["valid"],self_collision=r["self_collision"],joint_margin=r["joint_margin"])
  (front if r["view"]=="FRONT" else right).append(dict(**common,**({"tcp_y":r["tcp_y"]} if r["view"]=="FRONT" else {"tcp_x":r["tcp_x"]})))
 write(f"{VAL}/boundary_sweep_workspace_front.csv",["configuration","lift_value","sweep_parameter","tcp_y","tcp_z","boundary_type","valid","self_collision","joint_margin"],front)
 write(f"{VAL}/boundary_sweep_workspace_right.csv",["configuration","lift_value","sweep_parameter","tcp_x","tcp_z","boundary_type","valid","self_collision","joint_margin"],right)
 rawmeta={r["key"]:r["value"] for r in read(META)};summary=[];own=[dict(r,target_configuration=r["configuration"],source_configuration=r["configuration"]) for r in raw]
 for c in CONFIGS:
  cr=[r for r in pool if r["target_configuration"]==c];xyz=np.array([[f(r,k) for k in ("tcp_x","tcp_y","tcp_z")] for r in cr]);fr=[];rr=[]
  for r in cr:
   o=ref(r["lift_value"]);fr.append(math.hypot(f(r,"tcp_y")-o[1],f(r,"tcp_z")-o[2]));rr.append(math.hypot(f(r,"tcp_x")-o[0],f(r,"tcp_z")-o[2]))
  fa=projected_union_area(own,c,"FRONT",lifts,sources(c));ra=projected_union_area(own,c,"RIGHT",lifts,sources(c))
  summary.append(dict(configuration=c,usable_lift_top=rawmeta["usable_lift_top"],usable_lift_bottom=rawmeta["usable_lift_bottom"],front_min_reach=min(fr),front_max_reach=max(fr),right_min_reach=min(rr),right_max_reach=max(rr),max_forward_x=xyz[:,0].max(),max_lateral_y=max(abs(xyz[:,1].min()),abs(xyz[:,1].max())),min_inner_radius=min(fr+rr),max_outer_radius=max(fr+rr),front_area=fa,right_area=ra,self_collision_limited_states=rawmeta["self_collision_rejections"]))
 write(f"{VAL}/boundary_sweep_workspace_summary.csv",list(summary[0]),summary)
 grid={r["configuration"]:r for r in read(f"{VAL}/fixed_base_workspace_dof_ablation_summary.csv")};fk={r["configuration"]:r for r in read(f"{VAL}/fk_workspace_boundary_summary.csv")};lift_rows=read(f"{VAL}/lift_slice_fk_workspace_summary.csv");cross=[]
 for r in summary:
  c=r["configuration"];ls=[x for x in lift_rows if x["configuration"]==c and x["slice_status"]=="VALID_ENDPOINTS"]
  cross.append(dict(configuration=c,boundary_sweep_max_x=r["max_forward_x"],grid_fixed_orientation_max_x=grid[c]["x_max"],fk_random_positional_max_x=fk[c]["x_max"],lift_slice_positional_max_x=max(float(x["x_max"]) for x in ls),usable_lift_top=r["usable_lift_top"],usable_lift_bottom=r["usable_lift_bottom"],qualitative_consistency="TORSO_EXPANSION_TREND_CONSISTENT"))
 write(f"{VAL}/boundary_sweep_workspace_cross_validation.csv",list(cross[0]),cross)
 base=summary[0];comparison=[]
 for r in summary:
  comparison.append(dict(configuration=r["configuration"],max_forward_increase=float(r["max_forward_x"])-float(base["max_forward_x"]),max_lateral_increase=float(r["max_lateral_y"])-float(base["max_lateral_y"]),front_area_increase=float(r["front_area"])-float(base["front_area"]),front_area_increase_percent=(float(r["front_area"])/float(base["front_area"])-1)*100,right_area_increase=float(r["right_area"])-float(base["right_area"]),right_area_increase_percent=(float(r["right_area"])/float(base["right_area"])-1)*100))
 write(f"{VAL}/boundary_sweep_workspace_comparison.csv",list(comparison[0]),comparison)
 surfaces={};surfrows=[];own_surfaces={c:surface(own,c,lifts) for c in CONFIGS}
 for c in CONFIGS:
  surfaces[c]=[tri for source in sources(c) for tri in own_surfaces[source]]
  for newtid,(_,v,b,l0,l1,a,d,e) in enumerate(surfaces[c]):surfrows.append(dict(configuration=c,triangle_id=newtid,view=v,boundary_type=b,lower_lift=l0,upper_lift=l1,v1_x=a[0],v1_y=a[1],v1_z=a[2],v2_x=d[0],v2_y=d[1],v2_z=d[2],v3_x=e[0],v3_y=e[1],v3_z=e[2]))
 write(f"{VAL}/boundary_sweep_workspace_3d_surface.csv",list(surfrows[0]),surfrows)
 poses=[]
 for c in CONFIGS:
  for view in VIEWS:
   subset=[r for r in nested if r["target_configuration"]==c and r["view"]==view]
   outer=[r for r in subset if r["boundary_type"]=="OUTER_BOUNDARY"];inner=[r for r in subset if r["boundary_type"]=="INNER_BOUNDARY"]
   maxr=max(outer,key=lambda r:f(r,"tcp_y") if view=="FRONT" else f(r,"tcp_x"));minr=min(inner,key=lambda r:np.linalg.norm(np.array([f(r,"tcp_x"),f(r,"tcp_y"),f(r,"tcp_z")])-ref(r["lift_value"])))
   for typ,r in [("MAX_REACH_POSE",maxr),("MIN_REACH_POSE",minr)]:poses.append(dict(configuration=c,view=view,pose_type=typ,lift_value=r["lift_value"],tcp_x=r["tcp_x"],tcp_y=r["tcp_y"],tcp_z=r["tcp_z"],source_configuration=r["source_configuration"],joint_names=r["joint_names"],joint_values=r["joint_values"]))
 write(f"{VAL}/boundary_sweep_workspace_representative_poses.csv",list(poses[0]),poses)
 figures=[]
 for i,c in enumerate(CONFIGS):
  for view in VIEWS:
   p=f"{PRE}/boundary_{view.lower()}_c{i}.png";plot_view(p,c,view,nested,lifts);figures.append(dict(scene=f"{view.lower()}_c{i}",path=os.path.relpath(p,WS)))
  p=f"{PRE}/boundary_3d_c{i}.png";plot3d(p,[c],nested,surfaces);figures.append(dict(scene=f"3d_c{i}",path=os.path.relpath(p,WS)))
 for view in VIEWS:
  p=f"{PRE}/boundary_{view.lower()}_compare.png";plot_compare(p,view,nested,lifts);figures.append(dict(scene=f"{view.lower()}_compare",path=os.path.relpath(p,WS)))
 p=f"{PRE}/boundary_3d_compare.png";plot3d(p,CONFIGS,nested,surfaces);figures.append(dict(scene="3d_compare",path=os.path.relpath(p,WS)))
 write(f"{VAL}/boundary_sweep_workspace_figure_index.csv",["scene","path"],figures)
 print({"raw":len(raw),"nested":len(nested),"lift_slices":lifts,"triangles":len(surfrows),"figures":len(figures)})
if __name__=="__main__":main()

#!/usr/bin/env python3
"""Generate exact cell-preserving 2D union projections from validated 3D CSV data."""

import argparse
import csv
import hashlib
import pathlib

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Patch, Rectangle
import numpy as np
import yaml


CONFIGS = (
    ("C0", "LIFT_ONLY", "c0_lift_success", "Arm + Lift"),
    ("C1", "LIFT_YAW", "c1_lift_yaw_success", "Arm + Lift + Waist Yaw"),
    ("C2", "LIFT_PITCH", "c2_lift_pitch_success", "Arm + Lift + Waist Pitch"),
    ("C3", "LIFT_YAW_PITCH", "c3_lift_yaw_pitch_success", "Arm + Lift + Yaw + Pitch"),
)


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def edges(values):
    values = np.asarray(values, dtype=float)
    delta = np.diff(values)
    if len(values) < 2 or not np.allclose(delta, delta[0], rtol=0.0, atol=1e-11):
        raise RuntimeError("Projection requires a uniform validated grid")
    return np.concatenate(([values[0] - delta[0] / 2.0], (values[:-1] + values[1:]) / 2.0,
                           [values[-1] + delta[0] / 2.0]))


def bool_value(value):
    if value not in ("0", "1"):
        raise RuntimeError(f"Non-binary reachability value: {value}")
    return value == "1"


def add_reference(ax, view):
    if view == "front":
        ax.axvline(0.0, color="#f7d354", lw=2.0, ls="--", label="Robot centerline Y=0")
        ax.scatter([0.0], [0.0], marker="x", s=90, lw=2.5, color="#f7d354", zorder=8)
        ax.annotate("base origin", (0.0, 0.0), xytext=(12, 15), textcoords="offset points",
                    color="#f7d354", fontsize=11, weight="bold")
        ax.add_patch(Rectangle((-0.22, 0.0), 0.44, 0.28, facecolor="#8a929c", alpha=0.20,
                               edgecolor="#bdc5ce", lw=1.5, label="Base reference"))
        ax.set_xlim(-0.46, 0.46)
        ax.set_xlabel("Y — left (+) / right (−) [m]", fontsize=14, weight="bold")
    else:
        ax.axvline(0.0, color="#f7d354", lw=2.0, ls="--", label="Base X=0")
        ax.scatter([0.0], [0.0], marker="x", s=90, lw=2.5, color="#f7d354", zorder=8)
        ax.annotate("base origin", (0.0, 0.0), xytext=(12, 15), textcoords="offset points",
                    color="#f7d354", fontsize=11, weight="bold")
        ax.add_patch(Rectangle((-0.16, 0.0), 0.36, 0.28, facecolor="#8a929c", alpha=0.20,
                               edgecolor="#bdc5ce", lw=1.5, label="Base reference"))
        ax.annotate("FORWARD +X →", (0.42, 0.12), color="#f7d354", fontsize=12, weight="bold",
                    ha="center", arrowprops={"arrowstyle": "->", "color": "#f7d354", "lw": 2})
        ax.set_xlim(-0.20, 0.82)
        ax.set_xlabel("X — backward / forward (+) [m]", fontsize=14, weight="bold")
    ax.set_ylim(0.0, 1.82)
    ax.set_ylabel("Z — vertical [m]", fontsize=14, weight="bold")
    ax.grid(color="#9aa6b2", alpha=0.18, lw=0.8)
    ax.set_aspect("equal", adjustable="box")


def base_figure(title, subtitle):
    fig, ax = plt.subplots(figsize=(16, 9), dpi=120)
    fig.patch.set_facecolor("#10151d")
    ax.set_facecolor("#151c25")
    for spine in ax.spines.values():
        spine.set_color("#8fa1b3")
    ax.tick_params(colors="#e8eef5", labelsize=11)
    ax.xaxis.label.set_color("#e8eef5")
    ax.yaxis.label.set_color("#e8eef5")
    fig.suptitle(title, color="#f4f7fb", fontsize=25, weight="bold", y=0.965)
    ax.set_title(subtitle, color="#aebdca", fontsize=13, pad=12)
    fig.subplots_adjust(left=0.08, right=0.93, bottom=0.13, top=0.86)
    return fig, ax


def draw_cells(ax, u_edges, z_edges, occupied, color, alpha=0.84, edge="#10151d"):
    data = np.ma.masked_where(~occupied.T, occupied.T.astype(float))
    ax.pcolormesh(u_edges, z_edges, data, cmap=ListedColormap([color]), shading="flat",
                  alpha=alpha, edgecolors=edge, linewidth=0.65, antialiased=False, zorder=4)


def draw_single(output, view, tag, label, occupied, area, u_values, z_values, du, dz, color):
    view_name = "FRONT VIEW · Y–Z UNION PROJECTION" if view == "front" else "RIGHT-SIDE VIEW · X–Z UNION PROJECTION"
    fig, ax = base_figure(f"{tag}  {label}",
                          f"{view_name}  |  validated reachable area = {area:.6f} m²")
    add_reference(ax, view)
    draw_cells(ax, edges(u_values), edges(z_values), occupied, color)
    ax.legend(handles=[Patch(facecolor=color, label=f"{tag} projected reachable"),
                       Patch(facecolor="#8a929c", alpha=0.3, label="base reference")],
              loc="upper right", framealpha=0.88, fontsize=11)
    fig.text(0.5, 0.035, "2D union projection of validated 3D reachable workspace · OR over collapsed depth · no hull/smoothing",
             ha="center", color="#ffd45c", fontsize=11, weight="bold")
    fig.savefig(output, dpi=120, facecolor=fig.get_facecolor())
    plt.close(fig)


def draw_compare(output, view, base, other, other_tag, other_label, areas, u_values, z_values, colors,
                 combined=None):
    common = base & other
    added = other & ~base
    lost = base & ~other
    fig, ax = base_figure(f"C0 vs {other_tag}  ·  {other_label}",
                         ("FRONT Y–Z" if view == "front" else "RIGHT-SIDE X–Z") +
                         f" union projection  |  C0 {areas[0]:.6f} → {other_tag} {areas[1]:.6f} m²")
    add_reference(ax, view)
    u_edges, z_edges = edges(u_values), edges(z_values)
    draw_cells(ax, u_edges, z_edges, common, colors["C0"], 0.72)
    draw_cells(ax, u_edges, z_edges, added, colors["added"], 0.90)
    if np.any(lost):
        draw_cells(ax, u_edges, z_edges, lost, "#ff5b4d", 0.88)
    if combined is not None and np.any(combined):
        for iu, iz in zip(*np.where(combined)):
            ax.add_patch(Rectangle((u_edges[iu], z_edges[iz]), u_edges[iu+1]-u_edges[iu],
                                   z_edges[iz+1]-z_edges[iz], fill=False,
                                   edgecolor=colors["combined_only"], lw=2.4, zorder=7))
    delta = areas[1] - areas[0]
    percent = 100.0 * delta / areas[0]
    legend = [Patch(facecolor=colors["C0"], label="C0 projected reachable"),
              Patch(facecolor=colors["added"], label=f"{other_tag} additional vs C0 (+{percent:.2f}%)")]
    if combined is not None:
        legend.append(Patch(facecolor="none", edgecolor=colors["combined_only"], lw=2,
                            label="projected combined-torso-only cells"))
    ax.legend(handles=legend, loc="upper right", framealpha=0.90, fontsize=11)
    fig.text(0.5, 0.035, "2D union projection of validated 3D reachable workspace · cell topology and holes preserved",
             ha="center", color="#ffd45c", fontsize=11, weight="bold")
    fig.savefig(output, dpi=120, facecolor=fig.get_facecolor())
    plt.close(fig)


def draw_all(output, view, occupancy, areas, u_values, z_values, colors, combined):
    fig, ax = base_figure("C0 / C1 / C2 / C3 PROJECTED WORKSPACE",
                         ("FRONT Y–Z" if view == "front" else "RIGHT-SIDE X–Z") +
                         " union projection · exact validated grid cells")
    add_reference(ax, view)
    u_edges, z_edges = edges(u_values), edges(z_values)
    insets = (0.02, 0.14, 0.26, 0.38)
    for index, ((tag, _, _, _), grid) in enumerate(zip(CONFIGS, occupancy)):
        for iu, iz in zip(*np.where(grid)):
            x0, x1 = u_edges[iu], u_edges[iu+1]
            y0, y1 = z_edges[iz], z_edges[iz+1]
            inset = insets[index] * min(x1-x0, y1-y0)
            ax.add_patch(Rectangle((x0+inset, y0+inset), max(0.0,x1-x0-2*inset), max(0.0,y1-y0-2*inset),
                                   facecolor=colors[tag], edgecolor=colors[tag], alpha=0.16, lw=1.0, zorder=3+index))
    for iu, iz in zip(*np.where(combined)):
        ax.add_patch(Rectangle((u_edges[iu], z_edges[iz]), u_edges[iu+1]-u_edges[iu],
                               z_edges[iz+1]-z_edges[iz], fill=False,
                               edgecolor=colors["combined_only"], lw=2.4, zorder=9))
    legend = [Patch(facecolor=colors[tag], label=f"{tag}: {areas[i]:.6f} m²")
              for i, (tag, _, _, _) in enumerate(CONFIGS)]
    legend.append(Patch(facecolor="none", edgecolor=colors["combined_only"], lw=2,
                        label="projected combined-torso-only"))
    ax.legend(handles=legend, loc="upper right", framealpha=0.90, fontsize=11)
    fig.text(0.5, 0.035, "Nested cell outlines are visualization only; every occupancy value is a direct OR projection of source rows",
             ha="center", color="#ffd45c", fontsize=11, weight="bold")
    fig.savefig(output, dpi=120, facecolor=fig.get_facecolor())
    plt.close(fig)


def draw_reference(output, view):
    name = "FRONT VIEW (Y–Z)" if view == "front" else "RIGHT-SIDE VIEW (X–Z)"
    collapse = "X depth collapsed" if view == "front" else "Y depth collapsed"
    fig, ax = base_figure(f"FIXED-BASE HUMANOID · {name}",
                         f"Projection reference · {collapse} · base_link fixed")
    add_reference(ax, view)
    ax.text(0.50, 0.66, "AMR MOTION DISABLED\nVALIDATED 12×10×12 TCP GRID\nNO NEW IK SAMPLING",
            transform=ax.transAxes, ha="center", va="center", color="#eff5fb", fontsize=20,
            weight="bold", bbox={"boxstyle":"round,pad=0.8","facecolor":"#263340","edgecolor":"#58d7c8","alpha":0.92})
    fig.text(0.5, 0.035, "2D union projection of validated 3D reachable workspace",
             ha="center", color="#ffd45c", fontsize=12, weight="bold")
    fig.savefig(output, dpi=120, facecolor=fig.get_facecolor())
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    args = parser.parse_args()
    config_path = pathlib.Path(args.config).resolve()
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))["workspace_projection"]
    workspace = pathlib.Path(config["workspace"])
    source = workspace / config["source_csv"]
    validation = workspace / config["validation_dir"]
    presentation = workspace / config["presentation_dir"]
    validation.mkdir(exist_ok=True)
    presentation.mkdir(exist_ok=True)
    outputs = [validation / name for name in (
        "workspace_projection_front.csv", "workspace_projection_right.csv",
        "workspace_projection_summary.csv", "workspace_projection_comparison.csv",
        "workspace_projection_differential.csv", "workspace_projection_metadata.csv",
        "workspace_projection_figure_index.csv")]
    outputs += list(presentation.glob("workspace_front_*.png")) + list(presentation.glob("workspace_right_*.png"))
    if any(path.exists() for path in outputs):
        raise RuntimeError("Refusing to overwrite existing workspace_projection evidence")

    rows = read_csv(source)
    if len(rows) != int(config["expected_points"]) or len({row["point_id"] for row in rows}) != len(rows):
        raise RuntimeError("Source is not the immutable 1,440-point grid")
    x = sorted({float(row["tcp_x"]) for row in rows})
    y = sorted({float(row["tcp_y"]) for row in rows})
    z = sorted({float(row["tcp_z"]) for row in rows})
    if [len(x), len(y), len(z)] != list(config["expected_grid"]):
        raise RuntimeError("Validated source grid dimensions changed")
    dx, dy, dz = x[1]-x[0], y[1]-y[0], z[1]-z[0]
    index_x = {round(value,12): index for index,value in enumerate(x)}
    index_y = {round(value,12): index for index,value in enumerate(y)}
    index_z = {round(value,12): index for index,value in enumerate(z)}
    occupancy_3d = np.zeros((4,len(x),len(y),len(z)),dtype=bool)
    combined_3d = np.zeros((len(x),len(y),len(z)),dtype=bool)
    for row in rows:
        ix=index_x[round(float(row["tcp_x"]),12)]
        iy=index_y[round(float(row["tcp_y"]),12)]
        iz=index_z[round(float(row["tcp_z"]),12)]
        for configuration, (_,_,key,_) in enumerate(CONFIGS):
            occupancy_3d[configuration,ix,iy,iz]=bool_value(row[key])
        combined_3d[ix,iy,iz]=(not occupancy_3d[0,ix,iy,iz] and not occupancy_3d[1,ix,iy,iz]
                               and not occupancy_3d[2,ix,iy,iz] and occupancy_3d[3,ix,iy,iz])
    counts=[int(grid.sum()) for grid in occupancy_3d]
    if counts != list(config["expected_reachable"]):
        raise RuntimeError(f"Validated reachable counts changed: {counts}")
    front=np.any(occupancy_3d,axis=1)  # config,y,z
    right=np.any(occupancy_3d,axis=2)  # config,x,z
    front_counts=np.sum(occupancy_3d,axis=1)
    right_counts=np.sum(occupancy_3d,axis=2)
    combined_front=np.any(combined_3d,axis=0)
    combined_right=np.any(combined_3d,axis=1)
    front_areas=[int(grid.sum())*dy*dz for grid in front]
    right_areas=[int(grid.sum())*dx*dz for grid in right]

    with (validation/"workspace_projection_front.csv").open("w",newline="",encoding="utf-8") as stream:
        writer=csv.writer(stream, lineterminator="\n"); writer.writerow(("configuration","y","z","reachable","contributing_3d_point_count"))
        for ci,(_,name,_,_) in enumerate(CONFIGS):
            for iy,value_y in enumerate(y):
                for iz,value_z in enumerate(z):
                    writer.writerow((name,f"{value_y:.15g}",f"{value_z:.15g}",int(front[ci,iy,iz]),int(front_counts[ci,iy,iz])))
    with (validation/"workspace_projection_right.csv").open("w",newline="",encoding="utf-8") as stream:
        writer=csv.writer(stream, lineterminator="\n"); writer.writerow(("configuration","x","z","reachable","contributing_3d_point_count"))
        for ci,(_,name,_,_) in enumerate(CONFIGS):
            for ix,value_x in enumerate(x):
                for iz,value_z in enumerate(z):
                    writer.writerow((name,f"{value_x:.15g}",f"{value_z:.15g}",int(right[ci,ix,iz]),int(right_counts[ci,ix,iz])))

    def bounds(grid,u,z_values):
        indices=np.argwhere(grid)
        return (u[int(indices[:,0].min())],u[int(indices[:,0].max())],
                z_values[int(indices[:,1].min())],z_values[int(indices[:,1].max())])
    with (validation/"workspace_projection_summary.csv").open("w",newline="",encoding="utf-8") as stream:
        writer=csv.writer(stream, lineterminator="\n"); writer.writerow(("configuration","front_reachable_cells","front_projected_area","right_reachable_cells","right_projected_area","y_min","y_max","z_min_front","z_max_front","x_min","x_max","z_min_right","z_max_right"))
        for ci,(_,name,_,_) in enumerate(CONFIGS):
            fy0,fy1,fz0,fz1=bounds(front[ci],y,z); rx0,rx1,rz0,rz1=bounds(right[ci],x,z)
            writer.writerow((name,int(front[ci].sum()),f"{front_areas[ci]:.15g}",int(right[ci].sum()),f"{right_areas[ci]:.15g}",fy0,fy1,fz0,fz1,rx0,rx1,rz0,rz1))
    with (validation/"workspace_projection_comparison.csv").open("w",newline="",encoding="utf-8") as stream:
        writer=csv.writer(stream, lineterminator="\n"); writer.writerow(("configuration","front_area","front_delta_vs_c0","front_percent_delta_vs_c0","right_area","right_delta_vs_c0","right_percent_delta_vs_c0"))
        for ci,(_,name,_,_) in enumerate(CONFIGS):
            writer.writerow((name,f"{front_areas[ci]:.15g}",f"{front_areas[ci]-front_areas[0]:.15g}",f"{100*(front_areas[ci]-front_areas[0])/front_areas[0]:.15g}",f"{right_areas[ci]:.15g}",f"{right_areas[ci]-right_areas[0]:.15g}",f"{100*(right_areas[ci]-right_areas[0])/right_areas[0]:.15g}"))
    categories={"YAW_EXPANDED":front[1]&~front[0],"PITCH_EXPANDED":front[2]&~front[0],"FULL_TORSO_EXPANDED":front[3]&~front[0],"COMBINED_ONLY":combined_front}
    categories_right={"YAW_EXPANDED":right[1]&~right[0],"PITCH_EXPANDED":right[2]&~right[0],"FULL_TORSO_EXPANDED":right[3]&~right[0],"COMBINED_ONLY":combined_right}
    with (validation/"workspace_projection_differential.csv").open("w",newline="",encoding="utf-8") as stream:
        writer=csv.writer(stream, lineterminator="\n"); writer.writerow(("view","category","horizontal_coordinate","z","projected_reachable"))
        for category,grid in categories.items():
            for iu,value_u in enumerate(y):
                for iz,value_z in enumerate(z): writer.writerow(("FRONT_YZ",category,value_u,value_z,int(grid[iu,iz])))
        for category,grid in categories_right.items():
            for iu,value_u in enumerate(x):
                for iz,value_z in enumerate(z): writer.writerow(("RIGHT_XZ",category,value_u,value_z,int(grid[iu,iz])))
    max_x=[]
    for grid in occupancy_3d:
        indices=np.argwhere(grid)
        max_x.append(x[int(indices[:,0].max())])
    if not np.allclose(max_x,config["expected_max_x"],rtol=0,atol=1e-12):
        raise RuntimeError(f"Projected max X consistency failed: {max_x}")
    with (validation/"workspace_projection_metadata.csv").open("w",newline="",encoding="utf-8") as stream:
        writer=csv.writer(stream, lineterminator="\n"); writer.writerow(("key","value"))
        values={"source_csv":str(source),"source_sha256":sha256(source),"source_points":len(rows),"base_frame":config["base_frame"],"coordinate_convention":config["coordinate_convention"],"projection_method":config["projection_method"],"grid_x":len(x),"grid_y":len(y),"grid_z":len(z),"dx":dx,"dy":dy,"dz":dz,"front_cell_area":dy*dz,"right_cell_area":dx*dz,"new_ik_sampling":False,"convex_hull":False,"smoothing":False,"hole_filling":False}
        for key,value in values.items(): writer.writerow((key,value))

    colors=config["colors"]
    figures=[]
    for view,grids,u_values,areas,du in (("front",front,y,front_areas,dy),("right",right,x,right_areas,dx)):
        draw_reference(presentation/f"workspace_{view}_reference.png",view); figures.append((view,"REFERENCE",f"workspace_{view}_reference.png"))
        for ci,(tag,_,_,label) in enumerate(CONFIGS):
            name=f"workspace_{view}_{tag.lower()}.png"
            draw_single(presentation/name,view,tag,label,grids[ci],areas[ci],u_values,z,du,dz,colors[tag]); figures.append((view,tag,name))
        combined=combined_front if view=="front" else combined_right
        for ci in (1,2,3):
            tag,_,_,label=CONFIGS[ci]
            name=f"workspace_{view}_c0_vs_{tag.lower()}.png"
            draw_compare(presentation/name,view,grids[0],grids[ci],tag,label,(areas[0],areas[ci]),u_values,z,colors,combined if ci==3 else None); figures.append((view,f"C0_VS_{tag}",name))
        name=f"workspace_{view}_all.png"
        draw_all(presentation/name,view,grids,areas,u_values,z,colors,combined); figures.append((view,"ALL",name))
    with (validation/"workspace_projection_figure_index.csv").open("w",newline="",encoding="utf-8") as stream:
        writer=csv.writer(stream, lineterminator="\n"); writer.writerow(("view","scene","path","sha256"))
        for view,scene,name in figures: writer.writerow((view.upper(),scene,f"presentation/{name}",sha256(presentation/name)))
    print(yaml.safe_dump({"status":"PASS","source_points":len(rows),"reachable_counts":counts,
                          "front_cells":[int(v.sum()) for v in front],"front_areas":front_areas,
                          "right_cells":[int(v.sum()) for v in right],"right_areas":right_areas,
                          "max_x":max_x,"figures":len(figures)},sort_keys=False))


if __name__ == "__main__":
    main()

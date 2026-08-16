#!/usr/bin/env python3
"""Create paper figures/tables from the completed, immutable planning dataset.

This is a post-processing-only tool.  It never imports ROS or MoveIt and never
performs IK, planning, collision checking, or trajectory generation.
"""

from __future__ import annotations

import csv
import hashlib
import math
import shutil
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from PIL import Image


ROOT = Path(__file__).resolve().parents[2]
RUN_ROOT = ROOT / "validation/paper_main_simulation_dataset_v1/run_20260815_223216"
SUMMARY_ROOT = RUN_ROOT / "summaries"
REPLAY_FIGURES = ROOT / "validation/paper_result_rviz_replay_v1/run_20260816_125837/figures"
OUT = ROOT / "validation/paper_assets_v1"
FIGURES = OUT / "figures"
TABLES = OUT / "tables"
SOURCE_COPIES = OUT / "source_png_copies"
EXPECTED_RAW_SHA256 = "f95048fe3e48ebffd3d1f255242a22ea03f8505dd59aa19ae3658aef8b696cf8"

MODES = ["LOCKED", "YAW_ONLY", "PITCH_ONLY", "YAW_PITCH"]
MODE_LABELS = {
    "LOCKED": "Locked",
    "YAW_ONLY": "Yaw only",
    "PITCH_ONLY": "Pitch only",
    "YAW_PITCH": "Yaw + Pitch",
}
COLORS = {
    "LOCKED": "#4C566A",
    "YAW_ONLY": "#D08770",
    "PITCH_ONLY": "#5E81AC",
    "YAW_PITCH": "#A3BE8C",
}

SUMMARY_FILES = [
    "mode_workspace_boundary.csv",
    "mode_workspace_area_summary.csv",
    "joint_margin_summary.csv",
    "collision_clearance_summary.csv",
    "margin_threshold_sensitivity.csv",
    "failure_taxonomy.csv",
]
SOURCE_PNGS = [
    "yaw_pair_locked_failure.png",
    "yaw_pair_yaw_only_success.png",
    "pitch_pair_locked_failure.png",
    "pitch_pair_pitch_only_success.png",
]


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def snapshot_inputs() -> dict[str, str]:
    paths = [RUN_ROOT / "all_case_results.csv"]
    paths += [SUMMARY_ROOT / name for name in SUMMARY_FILES]
    paths += [REPLAY_FIGURES / name for name in SOURCE_PNGS]
    missing = [str(path) for path in paths if not path.is_file()]
    if missing:
        raise FileNotFoundError("Missing required inputs:\n" + "\n".join(missing))
    return {str(path.relative_to(ROOT)): sha256(path) for path in paths}


def write_hash_snapshot(path: Path, hashes: dict[str, str]) -> None:
    text = "".join(f"{digest}  {name}\n" for name, digest in sorted(hashes.items()))
    path.write_text(text, encoding="utf-8")


def markdown_table(frame: pd.DataFrame) -> str:
    def fmt(value: object) -> str:
        if pd.isna(value):
            return ""
        if isinstance(value, float):
            return f"{value:.6g}"
        return str(value).replace("|", "\\|")

    headers = [str(column) for column in frame.columns]
    rows = [[fmt(value) for value in row] for row in frame.itertuples(index=False, name=None)]
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    lines += ["| " + " | ".join(row) + " |" for row in rows]
    return "\n".join(lines) + "\n"


def save_table(frame: pd.DataFrame, stem: str, title: str, note: str) -> None:
    csv_path = TABLES / f"{stem}.csv"
    md_path = TABLES / f"{stem}.md"
    frame.to_csv(csv_path, index=False, quoting=csv.QUOTE_MINIMAL, lineterminator="\n")
    md_path.write_text(f"# {title}\n\n{note}\n\n{markdown_table(frame)}", encoding="utf-8")


def save_figure(fig: plt.Figure, stem: str) -> None:
    fig.savefig(FIGURES / f"{stem}.png", dpi=300, bbox_inches="tight", facecolor="white")
    fig.savefig(FIGURES / f"{stem}.pdf", bbox_inches="tight", facecolor="white")
    plt.close(fig)


def create_comparison_panel() -> None:
    for name in SOURCE_PNGS:
        shutil.copy2(REPLAY_FIGURES / name, SOURCE_COPIES / name)

    # All four RViz screenshots have the same 2780 x 1782 layout.  Use one
    # identical pixel crop so camera magnification and color rendering remain
    # unchanged.  The crop removes UI panels but preserves the full 3-D view.
    crop = (1175, 180, 2760, 1740)
    labels = [
        "(a) Locked — Yaw target: IK failure",
        "(b) Yaw-only −8°: extraction success",
        "(c) Locked — Pitch target: IK failure",
        "(d) Pitch-only +18°: extraction success",
    ]
    fig, axes = plt.subplots(2, 2, figsize=(14.0, 10.4), constrained_layout=True)
    for axis, name, label in zip(axes.flat, SOURCE_PNGS, labels):
        with Image.open(SOURCE_COPIES / name) as image:
            if image.size != (2780, 1782):
                raise ValueError(f"Unexpected source image size for {name}: {image.size}")
            panel = np.asarray(image.crop(crop).convert("RGB"))
        axis.imshow(panel)
        axis.set_title(label, fontsize=13, loc="left", pad=8)
        axis.set_axis_off()
    fig.suptitle("Same-target, single-axis posture selection (planning-only replay)", fontsize=16)
    save_figure(fig, "figure1_same_target_single_axis_recovery_2x2")


def create_workspace_boundary(boundary: pd.DataFrame) -> None:
    phase = boundary[boundary["phase"] == "PHASE1"].copy()
    expected = 16 * 2 * 4
    if len(phase) != expected:
        raise ValueError(f"Expected {expected} Phase-1 boundary rows, got {len(phase)}")

    fig, axes = plt.subplots(1, 2, figsize=(13.2, 6.2), constrained_layout=True)
    for axis, lift in zip(axes, [0.35, 0.40]):
        lift_rows = phase[np.isclose(phase["lift"], lift)]
        for mode in MODES:
            rows = lift_rows[lift_rows["mode"] == mode].sort_values("ray_angle_deg")
            if len(rows) != 16:
                raise ValueError(f"Expected 16 rays for lift={lift}, mode={mode}")
            theta = np.deg2rad(rows["ray_angle_deg"].to_numpy(float))
            radius = rows["last_success_distance"].to_numpy(float)
            x = radius * np.cos(theta)
            y = radius * np.sin(theta)
            x = np.r_[x, x[0]]
            y = np.r_[y, y[0]]
            axis.plot(x, y, marker="o", ms=3.5, lw=1.8, color=COLORS[mode], label=MODE_LABELS[mode])
            axis.fill(x, y, color=COLORS[mode], alpha=0.055)
        axis.axhline(0, color="#BBBBBB", lw=0.7)
        axis.axvline(0, color="#BBBBBB", lw=0.7)
        axis.set_aspect("equal", adjustable="box")
        axis.set_xlabel("Target offset ΔX from box center (m)")
        axis.set_ylabel("Target offset ΔY from box center (m)")
        axis.set_title(f"Lift = {lift:.2f} m")
        axis.grid(alpha=0.22)
        axis.legend(fontsize=9, loc="lower left")
    fig.suptitle("16-ray polygon estimate of the planning-only feasible boundary\n(not an exact continuous workspace)", fontsize=15)
    save_figure(fig, "figure2_workspace_boundary_16ray_estimate")


def create_area_plot(area: pd.DataFrame) -> pd.DataFrame:
    phase = area[area["phase"] == "PHASE1"].copy()
    phase["mode"] = pd.Categorical(phase["mode"], MODES, ordered=True)
    phase = phase.sort_values(["lift", "mode"])
    locked = phase[phase["mode"] == "LOCKED"].set_index("lift")["area_m2"]
    phase["increase_vs_locked_percent"] = [
        100.0 * (row.area_m2 / locked.loc[row.lift] - 1.0) for row in phase.itertuples()
    ]

    fig, axis = plt.subplots(figsize=(9.2, 5.7), constrained_layout=True)
    x = np.arange(2)
    width = 0.19
    lifts = [0.35, 0.40]
    for idx, mode in enumerate(MODES):
        rows = phase[phase["mode"] == mode].set_index("lift")
        values = [rows.loc[lift, "area_m2"] for lift in lifts]
        bars = axis.bar(x + (idx - 1.5) * width, values, width, label=MODE_LABELS[mode], color=COLORS[mode])
        if mode == "YAW_PITCH":
            for bar, lift in zip(bars, lifts):
                increase = float(rows.loc[lift, "increase_vs_locked_percent"])
                axis.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.0015,
                          f"+{increase:.1f}% vs Locked", ha="center", va="bottom", fontsize=8, rotation=90)
    axis.set_xticks(x, [f"Lift {lift:.2f} m" for lift in lifts])
    axis.set_ylabel("Estimated 16-ray polygon area (m²)")
    axis.set_title("Planning-only 16-ray polygon area estimate\n(not exact continuous workspace; increase is not a synergy claim)")
    axis.grid(axis="y", alpha=0.25)
    axis.legend(ncol=2)
    ymax = phase["area_m2"].max()
    axis.set_ylim(0, ymax * 1.28)
    save_figure(fig, "figure3_workspace_area_comparison")
    return phase


def create_margin_plot(margins: pd.DataFrame) -> pd.DataFrame:
    phase = margins[margins["phase"] == "PHASE1"].copy()
    phase["mode"] = pd.Categorical(phase["mode"], MODES, ordered=True)
    phase = phase.sort_values("mode")
    means = phase["arm_joint_1_7_min_margin_mean"].to_numpy(float)
    mins = phase["arm_joint_1_7_min_margin_min"].to_numpy(float)
    maxs = phase["arm_joint_1_7_min_margin_max"].to_numpy(float)
    lower = means - mins
    upper = maxs - means

    fig, axis = plt.subplots(figsize=(9.3, 5.7), constrained_layout=True)
    positions = np.arange(len(MODES))
    axis.bar(positions, means, color=[COLORS[m] for m in MODES], alpha=0.85)
    axis.errorbar(positions, means, yerr=np.vstack([lower, upper]), fmt="none", ecolor="black", capsize=6, lw=1.2)
    for x, row in zip(positions, phase.itertuples()):
        axis.text(x, row.arm_joint_1_7_min_margin_mean + 0.015,
                  f"mean {row.arm_joint_1_7_min_margin_mean:.3f}", ha="center", fontsize=9)
    axis.set_xticks(positions, [MODE_LABELS[mode] for mode in MODES])
    axis.set_ylabel("Arm joints 1–7 minimum limit margin (rad)")
    axis.set_title("Finite margins among stored feasible Phase-1 samples\n(error bars show stored minimum and maximum; IK-absence rows are excluded)")
    axis.grid(axis="y", alpha=0.25)
    save_figure(fig, "figure4_arm_joint_margin_comparison")
    return phase


def create_robustness_plot(clearance: pd.DataFrame, sensitivity: pd.DataFrame,
                           failures: pd.DataFrame) -> tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame]:
    clear = clearance[clearance["phase"] == "PHASE1"].copy()
    clear["mode"] = pd.Categorical(clear["mode"], MODES, ordered=True)
    clear = clear.sort_values("mode")
    sens = sensitivity.copy()
    fail = failures[failures["phase"] == "PHASE1"].copy()

    fig, axes = plt.subplots(2, 2, figsize=(14.2, 10.0), constrained_layout=True)

    # Clearance minima are the conservative stored values. Convert m -> mm.
    axis = axes[0, 0]
    x = np.arange(len(MODES))
    width = 0.34
    env_min = clear["environment_clearance_min"].to_numpy(float) * 1000.0
    self_min = clear["self_clearance_min"].to_numpy(float) * 1000.0
    axis.bar(x - width / 2, env_min, width, label="Environment minimum", color="#88C0D0")
    axis.bar(x + width / 2, self_min, width, label="Self-collision minimum", color="#BF616A")
    axis.set_yscale("log")
    axis.set_xticks(x, [MODE_LABELS[m] for m in MODES], rotation=15)
    axis.set_ylabel("Stored minimum clearance (mm, log scale)")
    axis.set_title("Nominal collision-clearance minima")
    axis.legend(fontsize=9)
    axis.grid(axis="y", which="both", alpha=0.22)
    yp_idx = MODES.index("YAW_PITCH")
    axis.annotate(f"{self_min[yp_idx]:.6f} mm\nnominal low-clearance",
                  (yp_idx + width / 2, self_min[yp_idx]), xytext=(yp_idx - 0.8, self_min[yp_idx] * 2.5),
                  arrowprops={"arrowstyle": "->", "lw": 0.8}, fontsize=8)

    threshold_order = ["nominal", "1deg", "2deg", "5deg", "10deg"]
    for axis, lift in zip([axes[0, 1], axes[1, 0]], [0.35, 0.40]):
        rows_lift = sens[np.isclose(sens["lift"], lift)]
        for mode in MODES:
            rows = rows_lift[rows_lift["mode"] == mode].set_index("threshold").reindex(threshold_order)
            axis.plot(threshold_order, rows["estimated_16_ray_area_m2"].to_numpy(float), marker="o",
                      color=COLORS[mode], label=MODE_LABELS[mode])
        axis.set_xlabel("Simulation arm-margin sensitivity threshold")
        axis.set_ylabel("Estimated 16-ray polygon area (m²)")
        axis.set_title(f"Lift {lift:.2f} m — sensitivity, not hardware tolerance")
        axis.grid(alpha=0.24)
        axis.legend(fontsize=8)

    axis = axes[1, 1]
    pivot = fail.pivot_table(index="mode", columns="classification", values="count", aggfunc="sum", fill_value=0)
    pivot = pivot.reindex(MODES, fill_value=0)
    bottom = np.zeros(len(MODES))
    class_colors = plt.cm.Set2(np.linspace(0, 1, max(1, len(pivot.columns))))
    for color, classification in zip(class_colors, pivot.columns):
        values = pivot[classification].to_numpy(float)
        axis.bar(np.arange(len(MODES)), values, bottom=bottom, label=classification, color=color)
        bottom += values
    axis.set_xticks(np.arange(len(MODES)), [MODE_LABELS[m] for m in MODES], rotation=15)
    axis.set_ylabel("Stored failed case rows (count)")
    axis.set_title("Failure taxonomy; gripper-envelope excluded from torso failure")
    axis.legend(fontsize=7, loc="upper left")
    axis.grid(axis="y", alpha=0.2)

    fig.suptitle("Planning-only robustness sensitivity and failure classification", fontsize=15)
    save_figure(fig, "figure5_robustness_and_failure_taxonomy")
    return clear, sens, fail


def create_tables(raw: pd.DataFrame, area_phase: pd.DataFrame, margin_phase: pd.DataFrame,
                  clearance_phase: pd.DataFrame) -> None:
    table1 = area_phase[["lift", "mode", "area_m2", "completed_rays", "expected_rays",
                         "increase_vs_locked_percent"]].copy()
    table1.columns = ["lift_m", "mode", "estimated_polygon_area_m2", "completed_rays",
                      "expected_rays", "increase_vs_locked_percent"]
    save_table(
        table1,
        "table1_lift_16ray_estimated_area",
        "Table 1. Lift-wise 16-ray estimated polygon area",
        "Area is a 16-ray polygon estimate, not an exact continuous workspace. Percentage is a direct YAW_PITCH-versus-LOCKED comparison, not a synergy claim.",
    )

    keys = [
        ("Yaw target", "PHASE1|R3_0.030|R3|0.030|0.400|LOCKED|0"),
        ("Yaw target", "PHASE1|R3_0.030|R3|0.030|0.400|YAW_ONLY|0"),
        ("Pitch target", "PHASE1|R3_0.020|R3|0.020|0.350|LOCKED|0"),
        ("Pitch target", "PHASE1|R3_0.020|R3|0.020|0.350|PITCH_ONLY|0"),
    ]
    indexed = raw.set_index("unique_key", drop=False)
    records: list[dict[str, object]] = []
    for pair, key in keys:
        if key not in indexed.index:
            raise KeyError(f"Missing ablation key: {key}")
        row = indexed.loc[key]
        locked = row["mode"] == "LOCKED"
        records.append({
            "target_pair": pair,
            "unique_key": key,
            "target_x_m": row["target_x"],
            "target_y_m": row["target_y"],
            "target_z_m": row["target_z"],
            "lift_m": row["lift"],
            "mode": row["mode"],
            "yaw_deg": 0.0 if locked else math.degrees(row["yaw_rad"]),
            "pitch_deg": 0.0 if locked else math.degrees(row["pitch_rad"]),
            "stored_arm_posture": bool(row["success"]),
            "success": bool(row["success"]),
            "arm_min_margin_rad": row["arm_joint_1_7_min_margin"],
            "environment_clearance_mm": row["environment_clearance"] * 1000.0,
            "self_clearance_mm": row["self_clearance"] * 1000.0,
            "failure_or_success_label": row["failure_label"],
            "classification": row["classification"],
        })
    table2 = pd.DataFrame(records)
    # Verify each pair differs only in posture-selection mode among common inputs.
    common = ["phase", "target_id", "ray", "ray_angle_deg", "distance_m", "target_x", "target_y",
              "target_z", "lift", "seed_bank"]
    for _, pair_rows in table2.groupby("target_pair"):
        raw_pair = raw[raw["unique_key"].isin(pair_rows["unique_key"])]
        if any(raw_pair[column].nunique(dropna=False) != 1 for column in common):
            raise ValueError(f"Same-target condition mismatch for {pair_rows.iloc[0]['target_pair']}")
    save_table(
        table2,
        "table2_same_target_axis_ablation",
        "Table 2. Same-target single-axis ablation",
        "LOCKED failure rows contain no stored Arm posture; yaw/pitch zero are experimental fixed conditions, not invented IK results. These pairs do not establish Yaw–Pitch synergy.",
    )

    merged = margin_phase.merge(clearance_phase, on=["phase", "mode"], validate="one_to_one")
    table3 = merged[[
        "mode", "arm_joint_1_7_min_margin_n", "arm_joint_1_7_min_margin_min",
        "arm_joint_1_7_min_margin_mean", "arm_joint_1_7_min_margin_max",
        "environment_clearance_min", "environment_clearance_mean", "environment_clearance_max",
        "self_clearance_min", "self_clearance_mean", "self_clearance_max",
    ]].copy()
    for column in [c for c in table3.columns if "clearance" in c]:
        table3[column] = table3[column] * 1000.0
        table3.rename(columns={column: f"{column}_mm"}, inplace=True)
    table3.rename(columns={
        "arm_joint_1_7_min_margin_n": "finite_feasible_margin_n",
        "arm_joint_1_7_min_margin_min": "arm_margin_min_rad",
        "arm_joint_1_7_min_margin_mean": "arm_margin_mean_rad",
        "arm_joint_1_7_min_margin_max": "arm_margin_max_rad",
    }, inplace=True)
    save_table(
        table3,
        "table3_joint_margin_collision_clearance",
        "Table 3. Joint margin and collision clearance",
        "Margins/clearances summarize finite stored feasible samples only. IK-absence failures are not treated as zero-margin successes. The 0.043687 mm YAW_PITCH self-clearance minimum is nominal low-clearance.",
    )


def create_audit(input_hashes: dict[str, str], raw: pd.DataFrame, boundary: pd.DataFrame,
                 area_phase: pd.DataFrame, margin_phase: pd.DataFrame,
                 clearance_phase: pd.DataFrame, sensitivity: pd.DataFrame,
                 failures: pd.DataFrame) -> None:
    generated = sorted(
        path for path in OUT.rglob("*")
        if path.is_file() and path.name not in {"paper_assets_audit.md", "SHA256SUMS.txt", "input_manifest_after.sha256"}
    )
    generated_hashes = {str(path.relative_to(OUT)): sha256(path) for path in generated}
    phase_counts = raw.groupby("phase").size().to_dict()
    min_self = float(clearance_phase["self_clearance_min"].min() * 1000.0)
    yp_min_self = float(clearance_phase.loc[clearance_phase["mode"] == "YAW_PITCH", "self_clearance_min"].iloc[0] * 1000.0)

    source_lines = "\n".join(f"- `{name}` — `{digest}`" for name, digest in sorted(input_hashes.items()))
    asset_lines = "\n".join(f"- `{name}` — `{digest}`" for name, digest in generated_hashes.items())
    text = f"""# paper_assets_v1 audit

## Scope and immutability

- Post-processing only: no ROS, MoveIt, IK, OMPL, collision-distance computation, controller, or trajectory execution was invoked.
- Raw results SHA-256: `{input_hashes['validation/paper_main_simulation_dataset_v1/run_20260815_223216/all_case_results.csv']}`.
- Expected raw SHA-256: `{EXPECTED_RAW_SHA256}`.
- Raw rows: {len(raw):,}; phase rows: {phase_counts}; duplicate `unique_key`: {int(raw['unique_key'].duplicated().sum())}.
- The four replay PNGs were copied byte-for-byte to `source_png_copies/`; all panel cropping was applied only to those copies in memory.

## Figure and table provenance

1. Figure 1 uses the four copied replay PNGs. Identical pixel crop `(left=1175, top=180, right=2760, bottom=1740)` preserves equal zoom and color rendering. The panels are two independent single-axis same-target comparisons; no Yaw–Pitch synergy is claimed.
2. Figure 2 uses `mode_workspace_boundary.csv` columns `phase`, `ray_angle_deg`, `lift`, `mode`, and `last_success_distance`. Coordinates are `ΔX=d cos(θ)`, `ΔY=d sin(θ)` and the 16 ordered endpoints are closed as a polygon.
3. Figure 3 and Table 1 use `mode_workspace_area_summary.csv`: `area_m2`, `lift`, `mode`, `completed_rays`, `expected_rays`. Increase is `100 × (A_YAW_PITCH/A_LOCKED − 1)` at the same Lift.
4. Figure 4 and Table 3 use `joint_margin_summary.csv`: finite-sample `arm_joint_1_7_min_margin_[n|min|mean|max]`. IK-absence rows have no finite margin and are excluded rather than converted to zero.
5. Figure 5 uses `collision_clearance_summary.csv`, `margin_threshold_sensitivity.csv`, and `failure_taxonomy.csv`. Clearance converts metres to millimetres by `mm = 1000 × m`. Threshold areas are simulation sensitivity criteria, not hardware tolerances. `GRIPPER_ENVELOPE_INFEASIBLE` remains separate from torso/posture failure.
6. Table 2 uses four exact rows from `all_case_results.csv`, including `unique_key`, target XYZ, ray/distance/Lift/seed, selected yaw/pitch, success, margins, clearances, `failure_label`, and `classification`. Same-target inputs were asserted equal within each pair.

## Included rows and exclusions

- Boundary figure: {len(boundary[boundary['phase'] == 'PHASE1'])} Phase-1 rows = 16 rays × 2 Lift values × 4 modes.
- Area table/figure: {len(area_phase)} Phase-1 aggregate rows.
- Margin and clearance: {len(margin_phase)} and {len(clearance_phase)} Phase-1 aggregate mode rows, respectively; their source sample counts are retained in Table 3.
- Sensitivity figure: {len(sensitivity)} stored rows (2 Lift values × 4 modes × 5 thresholds).
- Failure taxonomy figure: {len(failures)} Phase-1 classification rows; gripper-envelope counts are displayed separately.
- Same-target ablation table: 4 exact raw rows (two targets × locked/recovered mode).
- Phase 2 and Phase 3 rows are excluded from the 16-ray Phase-1 workspace figures/tables. They remain untouched in the source dataset.
- Failed cases without finite IK posture/margin/clearance are never imputed. Stored `nan`/`inf` values are not replaced with invented finite values.
- Minimum stored Phase-1 self-clearance: {min_self:.9f} mm; YAW_PITCH minimum: {yp_min_self:.9f} mm, classified here as simulation nominal low-clearance.

## Interpretation limits

- Do not claim an exact continuous workspace; the areas are 16-ray polygon estimates.
- Do not claim hardware robustness; all results are deterministic planning-only simulation data.
- Do not claim force closure or physical grasp force.
- Do not claim Yaw–Pitch synergy from the same-target panels or YAW_PITCH-versus-LOCKED area increase. Synergy requires both single-axis modes to fail at the same target while the combined mode succeeds, which these panels do not test.
- Within the stored finite Phase-1 feasible samples, Pitch-only and Yaw+Pitch have higher mean Arm limit margin than Yaw-only; this is not a hardware-performance claim.

## Input SHA-256

{source_lines}

## Generated-file SHA-256

The following hashes cover generated figures, tables, source copies, script, and the pre-generation input manifest. The audit file itself is hashed in `SHA256SUMS.txt` to avoid self-reference.

{asset_lines}
"""
    (OUT / "paper_assets_audit.md").write_text(text, encoding="utf-8")


def create_output_manifest() -> None:
    paths = sorted(path for path in OUT.rglob("*") if path.is_file() and path.name != "SHA256SUMS.txt")
    text = "".join(f"{sha256(path)}  {path.relative_to(OUT)}\n" for path in paths)
    (OUT / "SHA256SUMS.txt").write_text(text, encoding="utf-8")


def main() -> None:
    FIGURES.mkdir(parents=True, exist_ok=True)
    TABLES.mkdir(parents=True, exist_ok=True)
    SOURCE_COPIES.mkdir(parents=True, exist_ok=True)

    before = snapshot_inputs()
    raw_key = "validation/paper_main_simulation_dataset_v1/run_20260815_223216/all_case_results.csv"
    if before[raw_key] != EXPECTED_RAW_SHA256:
        raise RuntimeError(f"Raw CSV SHA-256 mismatch: {before[raw_key]}")
    write_hash_snapshot(OUT / "input_manifest_before.sha256", before)

    raw = pd.read_csv(RUN_ROOT / "all_case_results.csv")
    if len(raw) != 4320 or raw["unique_key"].duplicated().any():
        raise ValueError("Raw dataset row count or unique-key invariant failed")
    boundary = pd.read_csv(SUMMARY_ROOT / "mode_workspace_boundary.csv")
    area = pd.read_csv(SUMMARY_ROOT / "mode_workspace_area_summary.csv")
    margins = pd.read_csv(SUMMARY_ROOT / "joint_margin_summary.csv")
    clearance = pd.read_csv(SUMMARY_ROOT / "collision_clearance_summary.csv")
    sensitivity = pd.read_csv(SUMMARY_ROOT / "margin_threshold_sensitivity.csv")
    failures = pd.read_csv(SUMMARY_ROOT / "failure_taxonomy.csv")

    create_comparison_panel()
    create_workspace_boundary(boundary)
    area_phase = create_area_plot(area)
    margin_phase = create_margin_plot(margins)
    clearance_phase, sensitivity_used, failure_phase = create_robustness_plot(clearance, sensitivity, failures)
    create_tables(raw, area_phase, margin_phase, clearance_phase)

    after = snapshot_inputs()
    if before != after:
        changed = sorted(name for name in before if before[name] != after.get(name))
        raise RuntimeError(f"Input files changed during generation: {changed}")
    write_hash_snapshot(OUT / "input_manifest_after.sha256", after)
    create_audit(before, raw, boundary, area_phase, margin_phase, clearance_phase,
                 sensitivity_used, failure_phase)
    create_output_manifest()


if __name__ == "__main__":
    main()

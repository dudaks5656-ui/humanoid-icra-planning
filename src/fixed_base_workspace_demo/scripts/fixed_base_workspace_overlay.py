#!/usr/bin/env python3
"""Readable 2D presentation panel synchronized with the RViz demo scene.

The robot, TF, TCP marker and workspace clouds remain native RViz displays.
This companion window only renders validated CSV numbers and the current scene
label.  It has no publisher and no interface to planning or execution.
"""

import csv
import json
import pathlib
import re
import signal
import subprocess
import tkinter as tk

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Overlay(Node):
    def __init__(self):
        super().__init__("fixed_base_workspace_demo_overlay")
        self.declare_parameter("summary_csv", "")
        self.declare_parameter("contributions_csv", "")
        self.declare_parameter("runtime_status_json", "")
        self.summary_path = pathlib.Path(self.get_parameter("summary_csv").value)
        self.contribution_path = pathlib.Path(self.get_parameter("contributions_csv").value)
        self.runtime_path = pathlib.Path(self.get_parameter("runtime_status_json").value)
        self.summary = self._load_summary()
        self.contribution = self._one_row(self.contribution_path)
        self.scene = "INITIALIZING"
        self.create_subscription(
            String, "/fixed_base_workspace_demo/status", self._scene_callback,
            rclpy.qos.QoSProfile(
                depth=1,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

    @staticmethod
    def _one_row(path):
        with path.open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        if len(rows) != 1:
            raise RuntimeError(f"Expected one validated row: {path}")
        return rows[0]

    def _load_summary(self):
        with self.summary_path.open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        if len(rows) != 4:
            raise RuntimeError("Presentation overlay requires four summary rows")
        return {row["configuration"]: row for row in rows}

    def _scene_callback(self, message):
        self.scene = message.data

    def representative(self):
        try:
            return json.loads(self.runtime_path.read_text(encoding="utf-8"))
        except (FileNotFoundError, json.JSONDecodeError):
            return {}


class Panel:
    COLORS = {
        "ROBOT": "#f5f7fa",
        "C0": "#57a8ff",
        "C1": "#ffae36",
        "C2": "#f25bd7",
        "C3": "#39eadf",
        "COMBINED_C0": "#ff5a58",
        "COMBINED_C1": "#ff5a58",
        "COMBINED_C2": "#ff5a58",
        "COMBINED_C3": "#52f37a",
        "ANIMATION": "#52f37a",
        "FINAL": "#39eadf",
    }

    def __init__(self, node):
        self.node = node
        self.root = tk.Tk(className="FixedBaseWorkspacePresentationOverlay")
        self.root.title("Fixed-base Workspace Presentation Overlay")
        self.root.configure(bg="#101620")
        self.root.overrideredirect(True)
        self.root.attributes("-topmost", True)
        self.root.attributes("-alpha", 0.94)
        self.root.withdraw()

        self.accent = tk.Frame(self.root, bg="#39eadf", width=7)
        self.accent.pack(side="left", fill="y")
        body = tk.Frame(self.root, bg="#101620", padx=20, pady=14)
        body.pack(side="left", fill="both", expand=True)
        self.kicker = tk.Label(
            body, text="FIXED BASE  •  SAME 1,440 TCP POINTS",
            font=("DejaVu Sans", 9, "bold"), fg="#9fb2c8", bg="#101620", anchor="w")
        self.kicker.pack(fill="x")
        self.title = tk.Label(
            body, text="INITIALIZING", font=("DejaVu Sans", 18, "bold"),
            fg="#f5f7fa", bg="#101620", anchor="w", pady=4)
        self.title.pack(fill="x")
        self.detail = tk.Label(
            body, text="", font=("DejaVu Sans Mono", 12), justify="left",
            fg="#e9f1fa", bg="#101620", anchor="nw")
        self.detail.pack(fill="both", expand=True)
        self.safety = tk.Label(
            body,
            text="VISUALIZATION ONLY  •  NO EXECUTION\nNO CONTROLLER / ROS2_CONTROL / HARDWARE",
            font=("DejaVu Sans", 8, "bold"), fg="#ffd15a", bg="#101620",
            anchor="w", justify="left", pady=3)
        self.safety.pack(fill="x")
        self.last_scene = None
        self.root.after(100, self._find_rviz_and_show)
        self.root.after(50, self._tick)

    def _find_rviz_and_show(self):
        result = subprocess.run(
            ["xwininfo", "-root", "-tree"], text=True, capture_output=True, check=False)
        candidates = []
        for line in result.stdout.splitlines():
            if "rviz" not in line.lower():
                continue
            match = re.search(r"\b(0x[0-9a-fA-F]+)\b", line)
            if not match:
                continue
            detail = subprocess.run(
                ["xwininfo", "-id", match.group(1)], text=True, capture_output=True, check=False)
            width = re.search(r"Width:\s*(\d+)", detail.stdout)
            height = re.search(r"Height:\s*(\d+)", detail.stdout)
            x = re.search(r"Absolute upper-left X:\s*(-?\d+)", detail.stdout)
            y = re.search(r"Absolute upper-left Y:\s*(-?\d+)", detail.stdout)
            if width and height and x and y:
                values = tuple(map(int, (width.group(1), height.group(1), x.group(1), y.group(1))))
                if values[0] >= 800 and values[1] >= 600:
                    candidates.append(values)
        if not candidates:
            self.root.after(250, self._find_rviz_and_show)
            return
        width, height, x, y = max(candidates, key=lambda item: item[0] * item[1])
        panel_width = min(680, max(560, width // 3))
        panel_height = 360
        self.root.geometry(f"{panel_width}x{panel_height}+{x + 30}+{y + 105}")
        self.root.deiconify()
        self.root.lift()
        # RViz is sometimes discovered before GNOME finishes moving the window
        # from (0,0) to its final decorated position. Follow that placement so
        # the overlay remains inside the exact rectangle captured by ximagesrc.
        self.root.after(500, self._find_rviz_and_show)

    @staticmethod
    def _value(row, key, digits):
        return f"{float(row[key]):.{digits}f}"

    def _stats(self):
        s = self.node.summary
        names = ("LIFT_ONLY", "LIFT_YAW", "LIFT_PITCH", "LIFT_YAW_PITCH")
        tags = ("C0  ARM+LIFT", "C1  +WAIST YAW", "C2  +WAIST PITCH", "C3  +YAW+PITCH")
        lines = []
        for name, tag in zip(names, tags):
            row = s[name]
            increase = float(row["percent_delta_vs_lift_only"])
            suffix = "baseline" if name == "LIFT_ONLY" else f"+{increase:.2f}%"
            lines.append(
                f"{tag:<19} {int(row['reachable_points']):4d}/1440   "
                f"{float(row['targeted_workspace_volume']):.6f} m³   {suffix}"
            )
        return "\n".join(lines)

    def _content(self, scene):
        s = self.node.summary
        if scene in ("INITIALIZING", "WAITING"):
            return "INITIALIZING VERIFIED DATA", "Loading robot model and visualization state…"
        if scene == "ROBOT":
            return "FIXED-BASE HUMANOID", "AMR MOTION DISABLED\nBase position and orientation remain fixed."
        if scene in ("C0", "C1", "C2", "C3"):
            index = {"C0": 0, "C1": 1, "C2": 2, "C3": 3}[scene]
            name = ("LIFT_ONLY", "LIFT_YAW", "LIFT_PITCH", "LIFT_YAW_PITCH")[index]
            label = ("ARM + LIFT", "ADD WAIST YAW", "ADD WAIST PITCH", "FULL TORSO")[index]
            row = s[name]
            increase = float(row["percent_delta_vs_lift_only"])
            change = "BASELINE" if index == 0 else f"WORKSPACE INCREASE  +{increase:.2f}%"
            detail = (
                f"Reachable     {int(row['reachable_points'])} / 1440\n"
                f"Volume        {float(row['targeted_workspace_volume']):.6f} m³\n"
                f"Max X         {float(row['x_max']):.4f} m\n"
                f"{change}"
            )
            if index == 3:
                detail += "\nWAIST YAW + PITCH ENABLED"
            return f"{scene}  {label}", detail
        if scene.startswith("COMBINED_") or scene == "ANIMATION":
            status = self.node.representative()
            point_id = status.get("representative_point_id", 1360)
            xyz = status.get("representative_xyz", [0.7354167, 0.147, 1.03125])
            target = (
                f"SAME TARGET #{point_id}\n"
                f"TCP=({xyz[0]:.4f}, {xyz[1]:.4f}, {xyz[2]:.4f}) m"
            )
            if scene == "COMBINED_C3":
                return "C3 REACHABLE", f"{target}\nYAW + PITCH REQUIRED\nVALID C3 ROBOTSTATE DISPLAYED"
            if scene == "ANIMATION":
                return "C3 VISUALIZATION-ONLY ANIMATION", f"{target}\nNeutral → valid IK state\n101 collision-checked interpolation samples"
            config = scene[-2:]
            return f"{config} UNREACHABLE", f"{target}\nNo failed robot pose is fabricated.\nC0 / C1 / C2 FAIL  •  C3 PASS"
        if scene == "FINAL":
            c = self.node.contribution
            detail = self._stats() + (
                f"\nCombined-torso-only   {int(c['combined_torso_only_count'])} points   "
                f"{float(c['combined_torso_only_volume']):.6f} m³"
            )
            return "RESULT: FULL TORSO GIVES THE LARGEST WORKSPACE", detail
        return scene, self._stats()

    def _tick(self):
        if not rclpy.ok():
            self.root.destroy()
            return
        rclpy.spin_once(self.node, timeout_sec=0.0)
        scene = self.node.scene
        if scene != self.last_scene:
            title, detail = self._content(scene)
            color = self.COLORS.get(scene, "#f5f7fa")
            self.accent.configure(bg=color)
            self.title.configure(text=title, fg=color)
            self.detail.configure(text=detail)
            self.last_scene = scene
            self.root.lift()
        self.root.after(50, self._tick)

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    node = Overlay()
    panel = Panel(node)
    signal.signal(signal.SIGINT, lambda *_: panel.root.after(0, panel.root.destroy))
    signal.signal(signal.SIGTERM, lambda *_: panel.root.after(0, panel.root.destroy))
    try:
        panel.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

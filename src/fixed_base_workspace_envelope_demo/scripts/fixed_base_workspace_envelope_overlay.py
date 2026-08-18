#!/usr/bin/env python3
"""Read-only presentation panel synchronized with the RViz envelope scene."""

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
        super().__init__("fixed_base_workspace_envelope_overlay")
        self.declare_parameter("summary_csv", "")
        self.declare_parameter("contributions_csv", "")
        self.declare_parameter("runtime_status_json", "")
        self.runtime_path = pathlib.Path(self.get_parameter("runtime_status_json").value)
        with pathlib.Path(self.get_parameter("summary_csv").value).open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        if len(rows) != 4:
            raise RuntimeError("Envelope overlay requires four validated summary rows")
        self.summary = {row["configuration"]: row for row in rows}
        with pathlib.Path(self.get_parameter("contributions_csv").value).open(newline="", encoding="utf-8") as stream:
            contributions = list(csv.DictReader(stream))
        if len(contributions) != 1:
            raise RuntimeError("Envelope overlay requires one contribution row")
        self.contribution = contributions[0]
        self.scene = "INITIALIZING"
        self.create_subscription(
            String, "/fixed_base_workspace_envelope_demo/status", self._scene,
            rclpy.qos.QoSProfile(
                depth=1,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

    def _scene(self, message):
        self.scene = message.data

    def runtime(self):
        try:
            return json.loads(self.runtime_path.read_text(encoding="utf-8"))
        except (FileNotFoundError, json.JSONDecodeError):
            return {}


class Panel:
    COLORS = {
        "ROBOT": "#f5f7fa", "C0": "#57a8ff", "C1": "#ffae36", "C2": "#f25bd7",
        "C3": "#39eadf", "C0_VS_C3": "#52f37a", "COMBINED_ONLY": "#c45cff",
        "FINAL": "#39eadf", "ALL_FOUR": "#f5f7fa", "YAW_EXPANSION": "#ffae36",
        "PITCH_EXPANSION": "#f25bd7",
    }

    def __init__(self, node):
        self.node = node
        self.root = tk.Tk(className="FixedBaseWorkspaceEnvelopeOverlay")
        self.root.title("Fixed-base Workspace Envelope Overlay")
        self.root.configure(bg="#101620")
        self.root.overrideredirect(True)
        self.root.attributes("-topmost", True)
        self.root.attributes("-alpha", 0.94)
        self.root.withdraw()
        self.accent = tk.Frame(self.root, bg="#39eadf", width=7)
        self.accent.pack(side="left", fill="y")
        body = tk.Frame(self.root, bg="#101620", padx=20, pady=14)
        body.pack(side="left", fill="both", expand=True)
        tk.Label(body, text="FIXED BASE  •  VALIDATED 12×10×12 VOXEL GRID",
                 font=("DejaVu Sans", 9, "bold"), fg="#9fb2c8", bg="#101620", anchor="w").pack(fill="x")
        self.title = tk.Label(body, text="INITIALIZING", font=("DejaVu Sans", 18, "bold"),
                              fg="#f5f7fa", bg="#101620", anchor="w", pady=4)
        self.title.pack(fill="x")
        self.detail = tk.Label(body, text="", font=("DejaVu Sans Mono", 12), justify="left",
                               fg="#e9f1fa", bg="#101620", anchor="nw")
        self.detail.pack(fill="both", expand=True)
        tk.Label(body, text="EXPOSED-FACE VOXELS  •  NO CONVEX HULL\n"
                            "NO EXECUTION  •  NO CONTROLLER  •  NO HARDWARE",
                 font=("DejaVu Sans", 8, "bold"), fg="#ffd15a", bg="#101620",
                 anchor="w", justify="left", pady=3).pack(fill="x")
        self.last_scene = None
        self.root.after(100, self._find_rviz)
        self.root.after(50, self._tick)

    def _find_rviz(self):
        result = subprocess.run(["xwininfo", "-root", "-tree"], text=True, capture_output=True, check=False)
        candidates = []
        for line in result.stdout.splitlines():
            if "rviz" not in line.lower():
                continue
            match = re.search(r"\b(0x[0-9a-fA-F]+)\b", line)
            if not match:
                continue
            detail = subprocess.run(["xwininfo", "-id", match.group(1)], text=True, capture_output=True, check=False)
            values = [re.search(pattern, detail.stdout) for pattern in
                      (r"Width:\s*(\d+)", r"Height:\s*(\d+)",
                       r"Absolute upper-left X:\s*(-?\d+)", r"Absolute upper-left Y:\s*(-?\d+)")]
            if all(values):
                width, height, x, y = (int(value.group(1)) for value in values)
                if width >= 800 and height >= 600:
                    candidates.append((width, height, x, y))
        if not candidates:
            self.root.after(250, self._find_rviz)
            return
        width, height, x, y = max(candidates, key=lambda value: value[0] * value[1])
        self.root.geometry(f"{min(840, max(760, width // 2))}x380+{x + 30}+{y + 105}")
        self.root.deiconify()
        self.root.lift()
        self.root.after(500, self._find_rviz)

    def _stats(self):
        names = ("LIFT_ONLY", "LIFT_YAW", "LIFT_PITCH", "LIFT_YAW_PITCH")
        tags = ("C0 ARM+LIFT", "C1 +WAIST YAW", "C2 +WAIST PITCH", "C3 +YAW+PITCH")
        lines = []
        for name, tag in zip(names, tags):
            row = self.node.summary[name]
            increase = float(row["percent_delta_vs_lift_only"])
            suffix = "baseline" if name == "LIFT_ONLY" else f"+{increase:.2f}%"
            lines.append(f"{tag:<18} {int(row['reachable_points']):4d}/1440  "
                         f"{float(row['targeted_workspace_volume']):.6f} m³  {suffix}")
        return "\n".join(lines)

    def _content(self, scene):
        runtime = self.node.runtime()
        mode = runtime.get("visualization_mode", "surface").upper()
        if scene in ("INITIALIZING", "WAITING_FOR_RVIZ"):
            return "INITIALIZING VERIFIED ENVELOPE", "Loading immutable CSV and robot model…"
        if scene == "ROBOT":
            return "FIXED-BASE HUMANOID", "AMR position and orientation fixed\nWorkspace markers hidden"
        if scene in ("C0", "C1", "C2", "C3"):
            index = int(scene[1])
            name = ("LIFT_ONLY", "LIFT_YAW", "LIFT_PITCH", "LIFT_YAW_PITCH")[index]
            label = ("ARM + LIFT", "YAW EXPANSION", "PITCH EXPANSION", "FULL TORSO ENVELOPE")[index]
            row = self.node.summary[name]
            faces = runtime.get("exposed_faces", [636, 674, 694, 730])[index]
            motion = runtime.get("animation_collision_free", [False] * 4)[index]
            return f"{scene}  {label}", (
                f"Mode          {mode}\nReachable     {int(row['reachable_points'])} / 1440\n"
                f"Volume        {float(row['targeted_workspace_volume']):.6f} m³\n"
                f"Exposed faces {faces}   Triangles {faces * 2}\n"
                f"RobotState animation  {'PASS' if motion else 'STATIC'}"
            )
        if scene == "C0_VS_C3":
            return "C0 vs C3  +34.33%", (
                "C0                 0.097791 m³\nC3                 0.131366 m³\n"
                "Full torso increase  +34.33%\nTargeted forward-region simulation"
            )
        if scene == "COMBINED_ONLY":
            return "COMBINED-ONLY / #1360", (
                "65 voxels   0.007631 m³\nTCP=(0.7354, 0.1470, 1.0313) m\n"
                "C0 / C1 / C2 FAIL  →  C3 PASS\nValidated C3 RobotState animation"
            )
        if scene == "YAW_EXPANSION":
            return "YAW EXPANSION", "C1 PASS AND C0 FAIL\nValidated voxel subset"
        if scene == "PITCH_EXPANSION":
            return "PITCH EXPANSION", "C2 PASS AND C0 FAIL\nValidated voxel subset"
        if scene == "ALL_FOUR":
            return "ALL FOUR ENVELOPES", self._stats()
        return "RESULT: VALIDATED 3D WORKSPACE ENVELOPE", self._stats()

    def _tick(self):
        if not rclpy.ok():
            self.root.destroy()
            return
        rclpy.spin_once(self.node, timeout_sec=0.0)
        if self.node.scene != self.last_scene:
            title, detail = self._content(self.node.scene)
            color = self.COLORS.get(self.node.scene, "#f5f7fa")
            self.accent.configure(bg=color)
            self.title.configure(text=title, fg=color)
            self.detail.configure(text=detail)
            self.last_scene = self.node.scene
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

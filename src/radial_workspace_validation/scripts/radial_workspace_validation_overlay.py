#!/usr/bin/env python3
"""Presentation overlay synchronized with the radial RViz replay."""

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
        super().__init__("radial_workspace_validation_overlay")
        self.declare_parameter("summary_csv", "")
        self.declare_parameter("runtime_json", "")
        with pathlib.Path(self.get_parameter("summary_csv").value).open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        self.summary = {(row["configuration"], row["ray_name"]): row for row in rows}
        if len(self.summary) != 10:
            raise RuntimeError("Expected 5 rays x 2 configurations")
        self.runtime = pathlib.Path(self.get_parameter("runtime_json").value)
        self.scene = "INITIALIZING"
        self.create_subscription(String, "/radial_workspace_validation/status", self._status, 10)

    def _status(self, message):
        self.scene = message.data

    def payload(self):
        try:
            return json.loads(self.runtime.read_text(encoding="utf-8"))
        except (FileNotFoundError, json.JSONDecodeError):
            return {}


class Panel:
    def __init__(self, node):
        self.node = node
        self.root = tk.Tk(className="RadialWorkspaceValidationOverlay")
        self.root.overrideredirect(True)
        self.root.attributes("-topmost", True)
        self.root.attributes("-alpha", 0.95)
        self.root.configure(bg="#101620")
        self.root.withdraw()
        self.accent = tk.Frame(self.root, bg="#4de5d5", width=7)
        self.accent.pack(side="left", fill="y")
        body = tk.Frame(self.root, bg="#101620", padx=20, pady=14)
        body.pack(fill="both", expand=True)
        tk.Label(body, text="LIGHTWEIGHT RADIAL VALIDATION  •  base_link FRAME",
                 font=("DejaVu Sans", 9, "bold"), fg="#9fb2c8", bg="#101620", anchor="w").pack(fill="x")
        self.title = tk.Label(body, text="INITIALIZING", font=("DejaVu Sans", 18, "bold"),
                              fg="#4de5d5", bg="#101620", anchor="w")
        self.title.pack(fill="x", pady=5)
        self.detail = tk.Label(body, text="", font=("DejaVu Sans Mono", 12), justify="left",
                               fg="#edf5ff", bg="#101620", anchor="nw")
        self.detail.pack(fill="both", expand=True)
        tk.Label(body, text="PASS=GREEN  •  FAIL=RED  •  INTERNAL HOLE=PRESERVED\n"
                            "NO TRAJECTORY EXECUTION  •  NO CONTROLLER  •  NO HARDWARE",
                 font=("DejaVu Sans", 8, "bold"), fg="#ffd15a", bg="#101620", justify="left").pack(fill="x")
        self.last = None
        self.root.after(100, self.find_rviz)
        self.root.after(50, self.tick)

    def find_rviz(self):
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
        if candidates:
            width, height, x, y = max(candidates, key=lambda value: value[0] * value[1])
            self.root.geometry(f"920x390+{x + 30}+{y + 105}")
            self.root.deiconify()
            self.root.lift()
        self.root.after(500, self.find_rviz)

    def content(self):
        scene = self.node.scene
        if scene == "WAIT_FOR_RVIZ":
            return "PREPARING VERIFIED RViz SCENE", "Loading robot meshes, TF, C0/C3 envelope, and radial evidence…"
        payload = self.node.payload()
        config = payload.get("configuration", "LIFT_ONLY")
        ray = payload.get("ray", "FRONT")
        row = self.node.summary.get((config, ray), {})
        label = "C0 ARM + LIFT" if config == "LIFT_ONLY" else "C3 ARM + LIFT + YAW + PITCH"
        if scene == "COMPARE":
            a = self.node.summary[("LIFT_ONLY", "FRONT")]
            b = self.node.summary[("LIFT_YAW_PITCH", "FRONT")]
            return "C0 vs C3  •  FRONT", (
                f"C0 feasible  {a['first_feasible_distance'] or 'NONE'} – {a['last_feasible_distance'] or 'NONE'} m\n"
                f"C3 feasible  {b['first_feasible_distance'] or 'NONE'} – {b['last_feasible_distance'] or 'NONE'} m\n"
                f"Intervals    C0={a['feasible_interval_count']}  C3={b['feasible_interval_count']}\n"
                "Targeted radial boundary check; not a full workspace recomputation"
            )
        if scene == "HOLE":
            title = "INTERMEDIATE INFEASIBLE REGION" if int(row.get("hole_count", 0)) else "NO INTERNAL HOLE ON SELECTED RAY"
        elif scene.endswith("MIN"):
            title = f"{label}  •  FIRST FEASIBLE POSE"
        elif scene.endswith("MAX"):
            title = f"{label}  •  LAST FEASIBLE POSE"
        else:
            title = f"{label}  •  {ray} RAY"
        return title, (
            f"Ray                 {ray}\n"
            f"First / last        {row.get('first_feasible_distance') or 'NONE'} / {row.get('last_feasible_distance') or 'NONE'} m\n"
            f"Feasible intervals  {row.get('feasible_interval_count', '0')}\n"
            f"Internal holes      {row.get('hole_count', '0')}   largest={row.get('largest_hole') or '0'} m\n"
            f"PASS / FAIL         {row.get('pass_count', '0')} / {row.get('fail_count', '0')}"
        )

    def tick(self):
        if not rclpy.ok():
            self.root.destroy()
            return
        rclpy.spin_once(self.node, timeout_sec=0.0)
        if self.node.scene != self.last:
            title, detail = self.content()
            color = "#54a9ff" if self.node.scene.startswith("C0") else "#4de5d5"
            if self.node.scene == "HOLE":
                color = "#ff5a55"
            self.accent.configure(bg=color)
            self.title.configure(text=title, fg=color)
            self.detail.configure(text=detail)
            self.last = self.node.scene
            self.root.lift()
        self.root.after(50, self.tick)

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

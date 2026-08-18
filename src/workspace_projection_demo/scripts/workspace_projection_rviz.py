#!/usr/bin/env python3
"""Publish validated front/right workspace projections as RViz markers.

This node performs no IK.  It only renders the CSV projection generated from the
immutable 1,440-point DOF-ablation comparison.
"""

import csv
import json
import os
import time

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray


CONFIGS = ["LIFT_ONLY", "LIFT_YAW", "LIFT_PITCH", "LIFT_YAW_PITCH"]
SHORT = {
    "LIFT_ONLY": "C0 Arm + Lift",
    "LIFT_YAW": "C1 + Waist Yaw",
    "LIFT_PITCH": "C2 + Waist Pitch",
    "LIFT_YAW_PITCH": "C3 + Yaw + Pitch",
}
COLORS = {
    "LIFT_ONLY": (0.15, 0.57, 1.0),
    "LIFT_YAW": (1.0, 0.62, 0.11),
    "LIFT_PITCH": (0.88, 0.32, 0.82),
    "LIFT_YAW_PITCH": (0.13, 0.84, 0.78),
}


def rows(path):
    with open(path, newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def unique_spacing(values):
    ordered = sorted({round(float(value), 12) for value in values})
    gaps = [ordered[i + 1] - ordered[i] for i in range(len(ordered) - 1)]
    return min(gaps) if gaps else 0.01


class ProjectionNode(Node):
    def __init__(self):
        super().__init__("workspace_projection_rviz")
        for name, default in (
            ("front_csv", ""), ("right_csv", ""), ("summary_csv", ""),
            ("state_csv", ""), ("runtime_json", ""), ("scene", "front_compare"),
        ):
            self.declare_parameter(name, default)
        self.scene = self.get_parameter("scene").value
        allowed = {
            "front_c0", "front_c1", "front_c2", "front_c3", "front_compare",
            "right_c0", "right_c1", "right_c2", "right_c3", "right_compare",
        }
        if self.scene not in allowed:
            raise RuntimeError(f"Unsupported projection scene: {self.scene}")
        self.front = rows(self.get_parameter("front_csv").value)
        self.right = rows(self.get_parameter("right_csv").value)
        self.summary = {row["configuration"]: row for row in rows(self.get_parameter("summary_csv").value)}
        expected = {"LIFT_ONLY": 109, "LIFT_YAW": 118, "LIFT_PITCH": 113, "LIFT_YAW_PITCH": 119}
        actual = {
            config: sum(int(r["reachable"]) for r in self.front if r["configuration"] == config)
            for config in CONFIGS
        }
        if actual != expected:
            raise RuntimeError(f"Front projection occupancy drift: {actual}")

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.marker_pub = self.create_publisher(MarkerArray, "~/markers", qos)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.joint_names, self.joint_positions = self._load_state(self.get_parameter("state_csv").value)
        self.timer = self.create_timer(0.25, self.publish)
        self.started = time.time()
        self._write_runtime(actual)
        self.get_logger().info(
            f"WORKSPACE_PROJECTION scene={self.scene} source=validated_csv ik_sampling=NO"
        )

    def _load_state(self, path):
        if not path or not os.path.exists(path):
            return [], []
        state_rows = rows(path)
        if not state_rows:
            return [], []
        row = state_rows[0]
        return row["joint_names"].split(";"), [float(value) for value in row["joint_positions"].split(";")]

    def _write_runtime(self, counts):
        path = self.get_parameter("runtime_json").value
        if not path:
            return
        payload = {
            "scene": self.scene,
            "front_reachable_cells": counts,
            "new_ik_sampling": False,
            "trajectory_execution": False,
            "controller": False,
            "ros2_control": False,
            "hardware": False,
        }
        with open(path, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write("\n")

    @staticmethod
    def _marker(marker_id, marker_type, namespace):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rclpy.clock.Clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _cube_marker(self, marker_id, config, view, include, alpha=0.72):
        data = self.front if view == "front" else self.right
        marker = self._marker(marker_id, Marker.CUBE_LIST, f"{view}_{config}")
        color = COLORS[config]
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (*color, alpha)
        if view == "front":
            marker.scale.x = 0.012
            marker.scale.y = unique_spacing(r["y"] for r in data)
            marker.scale.z = unique_spacing(r["z"] for r in data)
            for row in data:
                if row["configuration"] == config and int(row["reachable"]) and include(row):
                    marker.points.append(Point(x=0.12, y=float(row["y"]), z=float(row["z"])))
        else:
            marker.scale.x = unique_spacing(r["x"] for r in data)
            marker.scale.y = 0.012
            marker.scale.z = unique_spacing(r["z"] for r in data)
            for row in data:
                if row["configuration"] == config and int(row["reachable"]) and include(row):
                    marker.points.append(Point(x=float(row["x"]), y=-0.18, z=float(row["z"])))
        return marker

    def _text_marker(self, marker_id, view, text):
        marker = self._marker(marker_id, Marker.TEXT_VIEW_FACING, f"{view}_caption")
        marker.pose.position.x = 0.10 if view == "front" else 0.36
        marker.pose.position.y = 0.62 if view == "front" else -0.20
        marker.pose.position.z = 1.80
        marker.scale.z = 0.075
        marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0
        marker.text = text
        return marker

    def _markers(self):
        view = "front" if self.scene.startswith("front") else "right"
        array = MarkerArray()
        if self.scene.endswith("compare"):
            base = {
                (float(r["y"]), float(r["z"])) if view == "front" else (float(r["x"]), float(r["z"]))
                for r in (self.front if view == "front" else self.right)
                if r["configuration"] == "LIFT_ONLY" and int(r["reachable"])
            }
            array.markers.append(self._cube_marker(0, "LIFT_ONLY", view, lambda _: True, 0.55))
            array.markers.append(self._cube_marker(
                1, "LIFT_YAW_PITCH", view,
                lambda row: ((float(row["y"]), float(row["z"])) if view == "front"
                             else (float(row["x"]), float(row["z"]))) not in base,
                0.90,
            ))
            caption = "C0 vs C3 UNION PROJECTION\nBlue: C0   Teal: C3 added\nValidated 3D grid; depth collapsed"
        else:
            index = int(self.scene[-1])
            config = CONFIGS[index]
            array.markers.append(self._cube_marker(0, config, view, lambda _: True, 0.72))
            summary = self.summary[config]
            area_key = "front_projected_area" if view == "front" else "right_projected_area"
            caption = f"{SHORT[config]}\nProjected area: {float(summary[area_key]):.6f} m^2\n2D union projection"
        array.markers.append(self._text_marker(20, view, caption))
        return array

    def publish(self):
        self.marker_pub.publish(self._markers())
        if self.joint_names:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = self.joint_positions
            self.joint_pub.publish(msg)


def main():
    rclpy.init()
    node = ProjectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import csv

import rclpy
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

WS = "/home/openarm/humanoid_sim_ws"
VAL = f"{WS}/validation"
CATEGORIES = ["BASELINE_C0", "YAW_UNIQUE", "PITCH_UNIQUE", "SINGLE_DOF_SHARED", "COMBINED_ONLY"]
NAMESPACES = {
    "BASELINE_C0": "baseline_c0",
    "YAW_UNIQUE": "yaw_unique",
    "PITCH_UNIQUE": "pitch_unique",
    "SINGLE_DOF_SHARED": "single_dof_shared",
    "COMBINED_ONLY": "combined_only",
}
COLORS = {
    "BASELINE_C0": (0.30, 0.47, 0.66, 0.28),
    "YAW_UNIQUE": (0.95, 0.66, 0.23, 0.78),
    "PITCH_UNIQUE": (0.85, 0.37, 0.73, 0.78),
    "SINGLE_DOF_SHARED": (0.35, 0.63, 0.31, 0.74),
    "COMBINED_ONLY": (0.89, 0.34, 0.34, 0.80),
}
SCENES = {
    "boundary_diff_all": CATEGORIES,
    "boundary_diff_yaw": ["BASELINE_C0", "YAW_UNIQUE", "SINGLE_DOF_SHARED"],
    "boundary_diff_pitch": ["BASELINE_C0", "PITCH_UNIQUE", "SINGLE_DOF_SHARED"],
    "boundary_diff_combined": ["BASELINE_C0", "COMBINED_ONLY"],
    "boundary_c0_vs_c3": CATEGORIES,
}


def read(name):
    with open(f"{VAL}/{name}", newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def point(row, prefix):
    value = Point()
    value.x = float(row[f"{prefix}_x"])
    value.y = float(row[f"{prefix}_y"])
    value.z = float(row[f"{prefix}_z"])
    return value


class DifferentialDemo(Node):
    def __init__(self):
        super().__init__("boundary_sweep_workspace_differential_rviz")
        self.declare_parameter("scene", "boundary_diff_all")
        self.scene = self.get_parameter("scene").value
        if self.scene not in SCENES:
            raise RuntimeError(f"Unknown scene {self.scene}; choose {sorted(SCENES)}")
        self.patches = read("boundary_sweep_workspace_differential_patches.csv")
        self.c0_surface = [
            row for row in read("boundary_sweep_workspace_3d_surface.csv") if row["configuration"] == "LIFT_ONLY"
        ]
        self.poses = read("boundary_sweep_workspace_representative_poses.csv")
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.marker_publisher = self.create_publisher(
            MarkerArray, "/boundary_sweep_workspace/differential_markers", qos
        )
        self.joint_publisher = self.create_publisher(JointState, "/joint_states", 1)
        self.create_timer(0.5, self.publish)

    def marker(self, category, rows, marker_id, alpha=None):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = NAMESPACES[category]
        marker.id = marker_id
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        red, green, blue, default_alpha = COLORS[category]
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = default_alpha if alpha is None else alpha
        marker.lifetime = Duration(seconds=1.2).to_msg()
        for row in rows:
            for prefix in ("v1", "v2", "v3"):
                marker.points.append(point(row, prefix))
        return marker

    def publish(self):
        markers = MarkerArray()
        active = SCENES[self.scene]
        marker_id = 0
        full_baseline = self.scene in {
            "boundary_diff_yaw", "boundary_diff_pitch", "boundary_diff_combined", "boundary_c0_vs_c3"
        }
        for category in active:
            if category == "BASELINE_C0" and full_baseline:
                rows = self.c0_surface
                alpha = 0.12 if self.scene == "boundary_diff_combined" else 0.20
            else:
                rows = [row for row in self.patches if row["category"] == category]
                alpha = None
            markers.markers.append(self.marker(category, rows, marker_id, alpha))
            marker_id += 1
        text = Marker()
        text.header.frame_id = "base_link"
        text.header.stamp = self.get_clock().now().to_msg()
        text.ns = "differential_scene_title"
        text.id = marker_id
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = 0.15
        text.pose.position.y = 0.72
        text.pose.position.z = 2.08
        text.scale.z = 0.075
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = self.scene.replace("boundary_", "").replace("_", " ").upper()
        text.lifetime = Duration(seconds=1.2).to_msg()
        markers.markers.append(text)
        self.marker_publisher.publish(markers)

        target = {
            "boundary_diff_yaw": "LIFT_YAW",
            "boundary_diff_pitch": "LIFT_PITCH",
        }.get(self.scene, "LIFT_YAW_PITCH")
        pose = next(
            row for row in self.poses
            if row["configuration"] == target and row["view"] == "RIGHT" and row["pose_type"] == "MAX_REACH_POSE"
        )
        joints = JointState()
        joints.header.stamp = self.get_clock().now().to_msg()
        joints.name = pose["joint_names"].split(";")
        joints.position = [float(value) for value in pose["joint_values"].split(";")]
        self.joint_publisher.publish(joints)


def main():
    rclpy.init()
    node = DifferentialDemo()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""RViz-only radial validation replay from immutable CSV evidence."""

import csv
import json
import math
import os
import pathlib
import time

import rclpy
from geometry_msgs.msg import Point
from moveit_msgs.msg import DisplayRobotState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray


def read_csv(path):
    with pathlib.Path(path).open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def as_bool(value):
    return str(value).strip() in ("1", "true", "True")


def rgba(r, g, b, a):
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


class RadialDemo(Node):
    def __init__(self):
        super().__init__("radial_workspace_validation_demo")
        self.declare_parameters("", [
            ("base_frame", "base_link"), ("comparison_csv", ""), ("points_csv", ""),
            ("intervals_csv", ""), ("holes_csv", ""), ("summary_csv", ""),
            ("states_csv", ""), ("metadata_csv", ""), ("runtime_json", ""),
            ("demo_scene", "auto"), ("selected_ray", "FRONT"),
            ("duration_scale", 1.0), ("publish_hz", 10.0),
            ("surface_alpha", 0.18), ("point_scale", 0.026),
        ])
        self.base_frame = self.get_parameter("base_frame").value
        self.scene_request = self.get_parameter("demo_scene").value
        self.selected_ray = self.get_parameter("selected_ray").value
        self.duration_scale = float(self.get_parameter("duration_scale").value)
        self.surface_alpha = float(self.get_parameter("surface_alpha").value)
        self.point_scale = float(self.get_parameter("point_scale").value)
        self.runtime_path = pathlib.Path(self.get_parameter("runtime_json").value)

        self.points = read_csv(self.get_parameter("points_csv").value)
        self.intervals = read_csv(self.get_parameter("intervals_csv").value)
        self.holes = read_csv(self.get_parameter("holes_csv").value)
        self.summary = read_csv(self.get_parameter("summary_csv").value)
        self.states = read_csv(self.get_parameter("states_csv").value)
        metadata = read_csv(self.get_parameter("metadata_csv").value)
        self.metadata = {row["key"]: row["value"] for row in metadata}
        comparison = read_csv(self.get_parameter("comparison_csv").value)
        if len(comparison) != 1440:
            raise RuntimeError("Demo requires immutable 1,440-point comparison")
        self.axes = [sorted({float(row[key]) for row in comparison}) for key in ("tcp_x", "tcp_y", "tcp_z")]
        if [len(axis) for axis in self.axes] != [12, 10, 12]:
            raise RuntimeError("Validated grid dimensions drifted")
        self.spacing = [axis[1] - axis[0] for axis in self.axes]
        self.occupancy = [set(), set()]
        for row in comparison:
            index = tuple(min(range(len(axis)), key=lambda i: abs(axis[i] - float(row[key])))
                          for axis, key in zip(self.axes, ("tcp_x", "tcp_y", "tcp_z")))
            if as_bool(row["c0_lift_success"]):
                self.occupancy[0].add(index)
            if as_bool(row["c3_lift_yaw_pitch_success"]):
                self.occupancy[1].add(index)
        if [len(value) for value in self.occupancy] != [833, 1119]:
            raise RuntimeError("Validated C0/C3 occupancy drifted")

        self.state_map = {}
        for row in self.states:
            names = row["joint_names"].split(";")
            positions = [float(value) for value in row["joint_positions"].split(";")]
            self.state_map[(row["configuration"], row["ray_name"], row["pose_role"])] = (names, positions)

        transient = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(MarkerArray, "/radial_workspace_validation/markers", transient)
        self.state_pub = self.create_publisher(DisplayRobotState, "/display_robot_state", transient)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.status_pub = self.create_publisher(String, "/radial_workspace_validation/status", transient)
        self.timeline = [
            ("C0_FRONT", 5.0), ("C0_MIN", 3.0), ("C0_MAX", 3.0),
            ("C3_FRONT", 5.0), ("C3_MIN", 3.0), ("C3_MAX", 3.0),
            ("HOLE", 7.0), ("COMPARE", 7.0),
        ]
        self.start = time.monotonic()
        self.last_scene = None
        self.create_timer(1.0 / float(self.get_parameter("publish_hz").value), self.tick)

    def marker(self, marker_id, namespace, marker_type, color):
        result = Marker()
        result.header.frame_id = self.base_frame
        result.header.stamp = self.get_clock().now().to_msg()
        result.ns = namespace
        result.id = marker_id
        result.type = marker_type
        result.action = Marker.ADD
        result.pose.orientation.w = 1.0
        result.color = color
        return result

    @staticmethod
    def point(xyz):
        return Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))

    def center(self, index):
        return tuple(self.axes[axis][index[axis]] for axis in range(3))

    def append_face(self, marker, center, direction):
        axis = direction // 2
        sign = 1.0 if direction % 2 == 0 else -1.0
        u_axis, v_axis = (axis + 1) % 3, (axis + 2) % 3
        face = list(center)
        face[axis] += sign * self.spacing[axis] * 0.5
        corners = []
        for u, v in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
            value = list(face)
            value[u_axis] += u * self.spacing[u_axis] * 0.5
            value[v_axis] += v * self.spacing[v_axis] * 0.5
            corners.append(value)
        for index in (0, 1, 2, 0, 2, 3):
            marker.points.append(self.point(corners[index]))

    def surface(self, occupied, marker_id, color):
        marker = self.marker(marker_id, "validated_envelope", Marker.TRIANGLE_LIST, color)
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0
        neighbors = ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1))
        for index in occupied:
            for direction, delta in enumerate(neighbors):
                neighbor = tuple(index[axis] + delta[axis] for axis in range(3))
                if neighbor not in occupied:
                    self.append_face(marker, self.center(index), direction)
        return marker

    def radial_markers(self, configuration, ray_name):
        rows = [row for row in self.points if row["configuration"] == configuration and row["ray_name"] == ray_name]
        rows.sort(key=lambda row: float(row["distance"]))
        array = []
        line = self.marker(20, "radial_ray", Marker.LINE_STRIP, rgba(1.0, 0.9, 0.2, 0.9))
        line.scale.x = 0.008
        line.points = [self.point((row["tcp_x"], row["tcp_y"], row["tcp_z"])) for row in (rows[0], rows[-1])]
        array.append(line)
        for status, color, marker_id in ((True, rgba(0.1, 1.0, 0.25, 1.0), 21),
                                         (False, rgba(1.0, 0.18, 0.15, 1.0), 22)):
            marker = self.marker(marker_id, "radial_samples", Marker.SPHERE_LIST, color)
            marker.scale.x = marker.scale.y = marker.scale.z = self.point_scale
            for row in rows:
                if as_bool(row["success"]) == status:
                    marker.points.append(self.point((row["tcp_x"], row["tcp_y"], row["tcp_z"])))
            array.append(marker)
        intervals = self.marker(23, "feasible_intervals", Marker.LINE_LIST, rgba(0.15, 1.0, 0.35, 1.0))
        intervals.scale.x = 0.018
        origin = [float(self.metadata[f"origin_{axis}"]) for axis in "xyz"]
        vector = [float(value) for value in self.metadata[f"ray_{ray_name}"].split(";")]
        for row in self.intervals:
            if row["configuration"] == configuration and row["ray_name"] == ray_name:
                for distance in (float(row["start_distance"]), float(row["end_distance"])):
                    intervals.points.append(self.point([origin[i] + vector[i] * distance for i in range(3)]))
        array.append(intervals)
        hole_marker = self.marker(24, "infeasible_holes", Marker.LINE_LIST, rgba(1.0, 0.05, 0.05, 1.0))
        hole_marker.scale.x = 0.025
        for row in self.holes:
            if row["configuration"] == configuration and row["ray_name"] == ray_name:
                for distance in (float(row["start_distance"]), float(row["end_distance"])):
                    hole_marker.points.append(self.point([origin[i] + vector[i] * distance for i in range(3)]))
        array.append(hole_marker)
        return array

    def representative_hole(self):
        if not self.holes:
            return "LIFT_YAW_PITCH", "FRONT"
        row = max(self.holes, key=lambda item: float(item["hole_length"]))
        return row["configuration"], row["ray_name"]

    def publish_state(self, configuration, ray_name, role):
        state = self.state_map.get((configuration, ray_name, role))
        if not state:
            return
        names, positions = state
        joint = JointState()
        joint.header.stamp = self.get_clock().now().to_msg()
        joint.name = names
        joint.position = positions
        display = DisplayRobotState()
        display.state.joint_state = joint
        self.state_pub.publish(display)
        self.joint_pub.publish(joint)

    def scene_at(self):
        if self.scene_request != "auto":
            return self.scene_request.upper()
        raw_elapsed = time.monotonic() - self.start
        # Publish a distinct preparation scene so the recorder timestamps C0 only after RViz/mesh/TF settle.
        if raw_elapsed < 4.0:
            return "WAIT_FOR_RVIZ"
        elapsed = raw_elapsed - 4.0
        for scene, duration in self.timeline:
            if elapsed < duration * self.duration_scale:
                return scene
            elapsed -= duration * self.duration_scale
        return "COMPARE"

    def tick(self):
        scene = self.scene_at()
        markers = MarkerArray()
        clear = self.marker(0, "clear", Marker.SPHERE, rgba(0, 0, 0, 0))
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        c0 = scene.startswith("C0") or scene == "WAIT_FOR_RVIZ"
        c3 = scene.startswith("C3")
        if scene == "HOLE":
            config, ray = self.representative_hole()
            c0, c3 = config == "LIFT_ONLY", config == "LIFT_YAW_PITCH"
        else:
            config, ray = ("LIFT_ONLY", "FRONT") if c0 else ("LIFT_YAW_PITCH", "FRONT")
        if scene == "COMPARE":
            markers.markers.append(self.surface(self.occupancy[0], 10, rgba(0.12, 0.55, 1.0, 0.17)))
            markers.markers.append(self.surface(self.occupancy[1] - self.occupancy[0], 11, rgba(0.18, 1.0, 0.38, 0.55)))
            markers.markers.extend(self.radial_markers("LIFT_ONLY", "FRONT"))
            markers.markers.extend(self.radial_markers("LIFT_YAW_PITCH", "FRONT"))
        else:
            markers.markers.append(self.surface(self.occupancy[0 if c0 else 1], 10,
                                                rgba(0.12, 0.55, 1.0, self.surface_alpha) if c0
                                                else rgba(0.12, 0.92, 0.88, self.surface_alpha)))
            markers.markers.extend(self.radial_markers(config, ray))
        self.marker_pub.publish(markers)
        if scene.endswith("MIN"):
            self.publish_state(config, ray, "FIRST_FEASIBLE")
        elif scene.endswith("MAX"):
            self.publish_state(config, ray, "LAST_FEASIBLE")
        else:
            self.publish_state(config, ray, "FIRST_FEASIBLE")
        self.status_pub.publish(String(data=scene))
        if scene != self.last_scene:
            payload = {
                "scene": scene, "configuration": config, "ray": ray,
                "elapsed_s": time.monotonic() - self.start,
                "trajectory_execution": False, "controller": False,
                "ros2_control": False, "hardware": False, "amr_motion": False,
            }
            temporary = self.runtime_path.with_suffix(".tmp")
            temporary.write_text(json.dumps(payload, indent=2), encoding="utf-8")
            os.replace(temporary, self.runtime_path)
            self.last_scene = scene


def main():
    rclpy.init()
    node = RadialDemo()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

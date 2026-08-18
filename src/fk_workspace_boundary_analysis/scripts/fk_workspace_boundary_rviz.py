#!/usr/bin/env python3
"""RViz-only replay of validated positional FK endpoint clouds."""

import csv
import json

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray


CONFIGS = ["LIFT_ONLY", "LIFT_YAW", "LIFT_PITCH", "LIFT_YAW_PITCH"]
COLORS = [(0.15, 0.57, 1.0), (1.0, 0.62, 0.11), (0.88, 0.32, 0.82), (0.13, 0.84, 0.78)]


class FKWorkspaceRviz(Node):
    def __init__(self):
        super().__init__("fk_workspace_boundary_rviz")
        self.declare_parameter("states_csv", "")
        self.declare_parameter("scene", "fk_compare")
        self.declare_parameter("runtime_json", "")
        self.scene = self.get_parameter("scene").value
        allowed = {"fk_c0", "fk_c1", "fk_c2", "fk_c3", "fk_compare"}
        if self.scene not in allowed:
            raise RuntimeError(f"Unsupported FK RViz scene: {self.scene}")
        with open(self.get_parameter("states_csv").value, newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        if len(rows) != 40000:
            raise RuntimeError(f"FK state source drift: {len(rows)}")
        self.valid = {config: [row for row in rows if row["configuration"] == config and row["valid"] == "1"]
                      for config in CONFIGS}
        if any(not rows for rows in self.valid.values()):
            raise RuntimeError("A configuration has no valid FK states")
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.marker_pub = self.create_publisher(MarkerArray, "~/markers", qos)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.timer = self.create_timer(0.25, self.publish)
        self._write_runtime()
        self.get_logger().info(f"FK_WORKSPACE_RVIZ scene={self.scene} IK=NO execution=NO")

    def _write_runtime(self):
        path = self.get_parameter("runtime_json").value
        if not path:
            return
        with open(path, "w", encoding="utf-8") as stream:
            json.dump({
                "scene": self.scene,
                "valid_counts": {key: len(value) for key, value in self.valid.items()},
                "new_ik_sampling": False,
                "trajectory_execution": False,
                "controller": False,
                "hardware": False,
            }, stream, indent=2, sort_keys=True)
            stream.write("\n")

    def marker(self, config, index, alpha):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "fk_endpoints"
        marker.id = index
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = 0.012
        marker.color.r, marker.color.g, marker.color.b = COLORS[index]
        marker.color.a = alpha
        marker.points = [Point(x=float(row["tcp_x"]), y=float(row["tcp_y"]), z=float(row["tcp_z"]))
                         for row in self.valid[config]]
        return marker

    def text_marker(self):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "fk_caption"; marker.id = 20
        marker.type = Marker.TEXT_VIEW_FACING; marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = 0.15; marker.pose.position.y = 0.65; marker.pose.position.z = 1.85
        marker.scale.z = 0.07
        marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0
        marker.text = "POSITIONAL FK WORKSPACE\nValid RobotState + self-collision check\nNO IK / NO EXECUTION"
        return marker

    def publish(self):
        array = MarkerArray()
        if self.scene == "fk_compare":
            for index, config in enumerate(CONFIGS):
                array.markers.append(self.marker(config, index, 0.24))
            pose_row = self.valid["LIFT_YAW_PITCH"][0]
        else:
            index = int(self.scene[-1])
            config = CONFIGS[index]
            array.markers.append(self.marker(config, index, 0.65))
            pose_row = self.valid[config][0]
        array.markers.append(self.text_marker())
        self.marker_pub.publish(array)
        state = JointState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.name = pose_row["joint_names"].split(";")
        state.position = [float(value) for value in pose_row["joint_values"].split(";")]
        self.joint_pub.publish(state)


def main():
    rclpy.init()
    node = FKWorkspaceRviz()
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

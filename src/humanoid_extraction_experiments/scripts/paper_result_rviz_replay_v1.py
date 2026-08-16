#!/usr/bin/env python3
"""CSV-only RViz replay for paper_main_simulation_dataset_v1.

This node never creates an IK solver, MoveGroup client, planner, controller, or
trajectory executor.  It republishes one stored joint posture and changes only
the stored Lift coordinate along the already validated 0.17 m descent/ascent.
"""

import argparse
import csv
import hashlib
import math
import os
import pathlib
import subprocess
import xml.etree.ElementTree as ET

import rclpy
import yaml
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, ObjectColor, PlanningScene, RobotState
from rclpy.duration import Duration as RclpyDuration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


LABELS = (
    "LOCKED_COMMON_SUCCESS",
    "YAW_ONLY_RECOVERY",
    "YAW_ONLY_RECOVERY_LOCKED_FAILURE",
    "PITCH_ONLY_RECOVERY",
    "PITCH_ONLY_RECOVERY_LOCKED_FAILURE",
    "YAW_PITCH_MAX_BOUNDARY",
    "YAW_PITCH_MIN_SELF_CLEARANCE",
    "GRIPPER_ENVELOPE_INFEASIBLE",
)
ARM_NAMES = [f"openarm_left_joint{i}" for i in range(1, 8)]
TCP_LINK = "openarm_left_hand_tcp"
FINGER_LINKS = ["openarm_left_left_finger", "openarm_left_right_finger"]


def read_csv(path):
    with pathlib.Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def finite(row, key):
    try:
        value = float(row[key])
        return value if math.isfinite(value) else None
    except (KeyError, TypeError, ValueError):
        return None


def lex(row):
    arm = finite(row, "arm_joint_1_7_min_margin") or -math.inf
    env = finite(row, "environment_clearance") or -math.inf
    self_clear = finite(row, "self_clearance") or -math.inf
    return arm, min(env, self_clear)


def select_representatives(rows, boundaries):
    index = {
        (r["phase"], r["ray"], r["distance_m"], r["lift"], r["seed_bank"], r["mode"]): r
        for r in rows
    }
    locked = max(
        (r for r in rows if r["phase"] == "PHASE1" and r["mode"] == "LOCKED" and r["success"] == "1"
         and float(r["distance_m"]) <= 0.0200001),
        key=lambda r: (*lex(r), -float(r["distance_m"])),
    )
    selected = {"LOCKED_COMMON_SUCCESS": locked}
    for mode, label in (("YAW_ONLY", "YAW_ONLY_RECOVERY"), ("PITCH_ONLY", "PITCH_ONLY_RECOVERY")):
        candidates = []
        for row in rows:
            if row["phase"] != "PHASE1" or row["mode"] != mode or row["success"] != "1":
                continue
            locked_row = index.get((row["phase"], row["ray"], row["distance_m"], row["lift"], row["seed_bank"], "LOCKED"))
            if locked_row and locked_row["success"] == "0" and locked_row["failure_label"] != "GRIPPER_ENVELOPE_INFEASIBLE":
                candidates.append(row)
        selected[label] = max(candidates, key=lex)

    recovery = selected["YAW_ONLY_RECOVERY"]
    selected["YAW_ONLY_RECOVERY_LOCKED_FAILURE"] = index[
        (recovery["phase"], recovery["ray"], recovery["distance_m"],
         recovery["lift"], recovery["seed_bank"], "LOCKED")
    ]
    recovery = selected["PITCH_ONLY_RECOVERY"]
    selected["PITCH_ONLY_RECOVERY_LOCKED_FAILURE"] = index[
        (recovery["phase"], recovery["ray"], recovery["distance_m"],
         recovery["lift"], recovery["seed_bank"], "LOCKED")
    ]

    boundary_candidates = []
    for boundary in boundaries:
        if boundary["phase"] != "PHASE1" or boundary["mode"] != "YAW_PITCH":
            continue
        try:
            distance = float(boundary["last_success_distance"])
        except ValueError:
            continue
        if not math.isfinite(distance):
            continue
        boundary_candidates.extend(
            row for row in rows
            if row["phase"] == "PHASE1" and row["mode"] == "YAW_PITCH" and row["ray"] == boundary["ray"]
            and row["lift"] == boundary["lift"] and row["success"] == "1"
            and abs(float(row["distance_m"]) - distance) < 1e-12
        )
    selected["YAW_PITCH_MAX_BOUNDARY"] = max(
        boundary_candidates, key=lambda r: (float(r["distance_m"]), *lex(r)))
    selected["YAW_PITCH_MIN_SELF_CLEARANCE"] = min(
        (r for r in rows if r["phase"] == "PHASE1" and r["mode"] == "YAW_PITCH" and r["success"] == "1"),
        key=lambda r: float(r["self_clearance"]),
    )
    selected["GRIPPER_ENVELOPE_INFEASIBLE"] = min(
        (r for r in rows if r["failure_label"] == "GRIPPER_ENVELOPE_INFEASIBLE"),
        key=lambda r: (r["phase"] != "PHASE1", ";" in r["collision_pairs"], float(r["distance_m"])),
    )
    return selected


def color(r, g, b, a=1.0):
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


class Replay(Node):
    def __init__(self, args):
        super().__init__("paper_result_rviz_replay_v1")
        self.args = args
        self.root = pathlib.Path(args.result_root)
        self.audit_path = pathlib.Path(args.selection_audit)
        self.audit = yaml.safe_load(self.audit_path.read_text())
        self.scene = yaml.safe_load(pathlib.Path(args.scene_config).read_text())
        self.rows = read_csv(self.root / "all_case_results.csv")
        self.boundaries = read_csv(self.root / "summaries/mode_workspace_boundary.csv")
        for required in (
            "summaries/selected_postures.csv", "summaries/joint_margin_summary.csv",
            "summaries/collision_clearance_summary.csv", "summaries/failure_taxonomy.csv"):
            if not (self.root / required).is_file():
                raise RuntimeError(f"Missing saved input: {required}")
        digest = hashlib.sha256((self.root / "all_case_results.csv").read_bytes()).hexdigest()
        if digest != self.audit["source_all_case_results_sha256"]:
            raise RuntimeError("all_case_results.csv SHA-256 no longer matches selection audit")
        automatic = select_representatives(self.rows, self.boundaries)
        for label in LABELS:
            expected = self.audit["cases"][label]["unique_key"]
            if automatic[label]["unique_key"] != expected:
                raise RuntimeError(f"Automatic selection drift for {label}: {automatic[label]['unique_key']} != {expected}")
        if args.case_label not in LABELS:
            raise RuntimeError(f"Unknown case label {args.case_label}; choose one of {', '.join(LABELS)}")
        self.label = args.case_label
        self.row = automatic[self.label]
        self.failure_visual = self.label in (
            "YAW_ONLY_RECOVERY_LOCKED_FAILURE", "PITCH_ONLY_RECOVERY_LOCKED_FAILURE",
            "GRIPPER_ENVELOPE_INFEASIBLE")
        self.envelope_visual = self.label == "GRIPPER_ENVELOPE_INFEASIBLE"
        selected_keys = {r["unique_key"] for r in read_csv(self.root / "summaries/selected_postures.csv")}
        if not self.failure_visual and self.row["unique_key"] not in selected_keys:
            raise RuntimeError("Successful replay key is absent from selected_postures.csv")

        self.urdf_xml = subprocess.run(
            ["xacro", args.robot_xacro], check=True, text=True, capture_output=True).stdout
        self.validate_robot_model()
        self.validate_saved_row()
        self.qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.scene_pub = self.create_publisher(PlanningScene, "/paper_result_replay/planning_scene", self.qos)
        self.marker_pub = self.create_publisher(MarkerArray, "/paper_result_replay/markers", self.qos)
        self.tf_buffer = Buffer(cache_time=RclpyDuration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.frames = self.make_frames()
        self.frame_index = 0
        self.lift_by_stamp = {}
        self.reference_tcp = None
        self.max_tcp_xy_error = self.max_tcp_z_error = self.max_tcp_orientation_error = 0.0
        self.runtime_path = pathlib.Path(args.runtime_audit)
        self.write_runtime("STARTED")
        self.timer = self.create_timer(0.1 / max(0.05, args.playback_speed), self.tick)
        self.get_logger().info(f"CSV_ONLY_REPLAY label={self.label} key={self.row['unique_key']} frames={len(self.frames)}")

    def validate_robot_model(self):
        root = ET.fromstring(self.urdf_xml)
        self.urdf_joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
        links = {link.attrib["name"] for link in root.findall("link")}
        required = ["lift_joint", "waist_yaw_joint", "waist_pitch_joint", *ARM_NAMES,
                    "openarm_left_finger_joint1", "openarm_right_finger_joint1"]
        missing = [name for name in required if name not in self.urdf_joints]
        if missing or TCP_LINK not in links:
            raise RuntimeError(f"CSV/RobotModel name mismatch: joints={missing}, tcp_present={TCP_LINK in links}")
        axis = self.urdf_joints["lift_joint"].find("axis").attrib.get("xyz", "")
        if [float(v) for v in axis.split()] != [0.0, 0.0, -1.0]:
            raise RuntimeError(f"Unexpected Lift axis: {axis}")

    def validate_saved_row(self):
        if self.failure_visual:
            if any(finite(self.row, name) is not None for name in ARM_NAMES):
                raise RuntimeError("Envelope failure unexpectedly contains a stored Arm posture")
            return
        values = {
            "lift_joint": float(self.row["lift"]),
            "waist_yaw_joint": float(self.row["yaw_rad"]),
            "waist_pitch_joint": float(self.row["pitch_rad"]),
            **{name: float(self.row[name]) for name in ARM_NAMES},
        }
        for name, value in values.items():
            limit = self.urdf_joints[name].find("limit")
            if limit is None:
                continue
            lower, upper = float(limit.attrib["lower"]), float(limit.attrib["upper"])
            if value < lower - 1e-12 or value > upper + 1e-12:
                raise RuntimeError(f"Stored {name}={value} violates URDF [{lower},{upper}]")

    def make_frames(self):
        if self.failure_visual:
            stage = ("GRASP_CONFIGURATION_IK_FAILURE"
                     if self.label in ("YAW_ONLY_RECOVERY_LOCKED_FAILURE",
                                       "PITCH_ONLY_RECOVERY_LOCKED_FAILURE")
                     else "PHYSICAL_FEASIBILITY_FAILURE")
            return [{"stage": stage, "lift": None, "attached": False, "finger": 0.044}]
        grasp = float(self.row["lift"])
        start = grasp - 0.17
        if start < -1e-12:
            raise RuntimeError("Stored Lift cannot replay a 0.17 m approach within limits")
        frames = []
        frames += [{"stage": "TOP_APPROACH_HOLD", "lift": start, "attached": False, "finger": 0.044}] * 20
        frames += [{"stage": "LIFT_VERTICAL_DESCENT", "lift": start + 0.001 * i,
                    "attached": False, "finger": 0.044} for i in range(171)]
        frames += [{"stage": "GRASP_HOLD", "lift": grasp, "attached": False,
                    "finger": float(self.scene["task"]["q_contact_50mm"])}] * 20
        frames += [{"stage": "ATTACHED_HOLD", "lift": grasp, "attached": True,
                    "finger": float(self.scene["task"]["q_contact_50mm"])}] * 20
        frames += [{"stage": "LIFT_ACTUATED_CLEARANCE", "lift": grasp - 0.001 * i,
                    "attached": True, "finger": float(self.scene["task"]["q_contact_50mm"])} for i in range(171)]
        return frames

    def joint_state(self, frame):
        msg = JointState(); msg.header.stamp = self.get_clock().now().to_msg()
        if self.failure_visual:
            return msg
        msg.name = ["lift_joint", "waist_yaw_joint", "waist_pitch_joint", *ARM_NAMES,
                    "openarm_left_finger_joint1", "openarm_right_finger_joint1"]
        msg.position = [frame["lift"], float(self.row["yaw_rad"]), float(self.row["pitch_rad"]),
                        *[float(self.row[name]) for name in ARM_NAMES], frame["finger"], 0.044]
        self.lift_by_stamp[(msg.header.stamp.sec, msg.header.stamp.nanosec)] = frame["lift"]
        if len(self.lift_by_stamp) > 1000:
            self.lift_by_stamp.pop(next(iter(self.lift_by_stamp)))
        return msg

    def collision_object(self, object_id, dimensions, xyz, operation=CollisionObject.ADD, frame="world", orientation=None):
        obj = CollisionObject(); obj.header.frame_id = frame; obj.id = object_id; obj.operation = operation
        if operation == CollisionObject.REMOVE:
            return obj
        shape = SolidPrimitive(); shape.type = SolidPrimitive.BOX; shape.dimensions = list(dimensions)
        pose = Pose(); pose.position.x, pose.position.y, pose.position.z = xyz; pose.orientation.w = 1.0
        if orientation is not None:
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = orientation
        obj.primitives = [shape]; obj.primitive_poses = [pose]
        return obj

    def box_objects(self):
        c = self.scene["box"]["center_xyz"]; w = self.scene["box"]["interior_width"]
        d = self.scene["box"]["interior_depth"]; h = self.scene["box"]["interior_height"]
        t = self.scene["box"]["wall_thickness"]; floor = self.scene["box"]["floor_thickness"]
        return [
            self.collision_object("box_bottom", [d + 2*t, w + 2*t, floor], [c[0], c[1], c[2]-h/2-floor/2]),
            self.collision_object("box_left_wall", [d, t, h], [c[0], c[1]+w/2+t/2, c[2]]),
            self.collision_object("box_right_wall", [d, t, h], [c[0], c[1]-w/2-t/2, c[2]]),
            self.collision_object("box_back_wall", [t, w+2*t, h], [c[0]+d/2+t/2, c[1], c[2]]),
            self.collision_object("box_front_wall", [t, w+2*t, h], [c[0]-d/2-t/2, c[1], c[2]]),
        ]

    def publish_scene(self, frame, joints):
        scene = PlanningScene(); scene.is_diff = True; scene.robot_state = RobotState(); scene.robot_state.is_diff = True
        scene.robot_state.joint_state = joints
        scene.world.collision_objects = self.box_objects()
        target = [float(self.row[k]) for k in ("target_x", "target_y", "target_z")]
        size = self.scene["target"]["size_xyz"]
        if frame["attached"] and not self.failure_visual:
            scene.world.collision_objects.append(self.collision_object("target_object", size, target, CollisionObject.REMOVE))
            attached = AttachedCollisionObject(); attached.link_name = TCP_LINK; attached.touch_links = FINGER_LINKS
            offset = float(self.scene["task"]["tcp_to_grasp_center"]) + float(self.scene["task"]["grasp_height_above_object_bottom"]) - size[2]/2
            attached.object = self.collision_object("target_object", size, [0.0, 0.0, offset], frame=TCP_LINK,
                                                    orientation=[0.0, -1.0, 0.0, 0.0])
            scene.robot_state.attached_collision_objects = [attached]
        else:
            scene.world.collision_objects.append(self.collision_object("target_object", size, target))
            removal = AttachedCollisionObject(); removal.link_name = TCP_LINK
            removal.object = self.collision_object("target_object", size, [0, 0, 0], CollisionObject.REMOVE, TCP_LINK)
            scene.robot_state.attached_collision_objects = [removal]
        scene.object_colors = [ObjectColor(id="target_object", color=self.case_color())]
        self.scene_pub.publish(scene)

    def case_color(self):
        if self.failure_visual:
            return color(0.9, 0.05, 0.05, 0.9)
        if self.label == "YAW_PITCH_MIN_SELF_CLEARANCE":
            return color(1.0, 0.45, 0.0, 0.95)
        return color(0.05, 0.85, 0.2, 0.95)

    def cube_marker(self, marker_id, namespace, dimensions, xyz, rgba):
        marker = Marker(); marker.header.frame_id = "world"; marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace; marker.id = marker_id; marker.type = Marker.CUBE; marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0; marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = xyz
        marker.scale.x, marker.scale.y, marker.scale.z = dimensions; marker.color = rgba
        return marker

    def markers(self, frame):
        array = MarkerArray(); c = self.scene["box"]["center_xyz"]; w = self.scene["box"]["interior_width"]
        d = self.scene["box"]["interior_depth"]; h = self.scene["box"]["interior_height"]
        t = self.scene["box"]["wall_thickness"]; floor = self.scene["box"]["floor_thickness"]
        boxes = [([d+2*t,w+2*t,floor],[c[0],c[1],c[2]-h/2-floor/2],"bottom"),
                 ([d,t,h],[c[0],c[1]+w/2+t/2,c[2]],"left"),([d,t,h],[c[0],c[1]-w/2-t/2,c[2]],"right"),
                 ([t,w+2*t,h],[c[0]+d/2+t/2,c[1],c[2]],"back"),([t,w+2*t,h],[c[0]-d/2-t/2,c[1],c[2]],"front")]
        for i,(dim,pos,name) in enumerate(boxes):
            highlight = self.envelope_visual and name == "front"
            array.markers.append(self.cube_marker(i,"box",dim,pos,color(0.95,0.1,0.1,0.65) if highlight else color(0.3,0.45,0.65,0.28)))
        target = [float(self.row[k]) for k in ("target_x","target_y","target_z")]
        if frame["attached"] and frame["lift"] is not None:
            target[2] += float(self.row["lift"]) - frame["lift"]
        array.markers.append(self.cube_marker(20,"target",self.scene["target"]["size_xyz"],target,self.case_color()))
        if self.envelope_visual:
            envelope_file = self.root / "raw/gripper_swept_envelope.csv"
            envelope = read_csv(envelope_file); center = self.scene["box"]["center_xyz"]
            mins = [min(float(r[k]) for r in envelope) for k in ("min_x","min_y","min_z")]
            maxs = [max(float(r[k]) for r in envelope) for k in ("max_x","max_y","max_z")]
            shift = [target[0]-center[0],target[1]-center[1],0.0]
            pos = [(mins[i]+maxs[i])/2+shift[i] for i in range(3)];dim=[maxs[i]-mins[i] for i in range(3)]
            array.markers.append(self.cube_marker(21,"gripper_swept_envelope",dim,pos,color(1.0,0.3,0.0,0.3)))
        yaw=self.row["yaw_rad"] if finite(self.row,"yaw_rad") is not None else "N/A"
        pitch=self.row["pitch_rad"] if finite(self.row,"pitch_rad") is not None else "N/A"
        if self.label == "YAW_ONLY_RECOVERY_LOCKED_FAILURE":
            heading = "SAME TARGET — LOCKED FAILURE"
        elif self.label == "PITCH_ONLY_RECOVERY_LOCKED_FAILURE":
            heading = "SAME TARGET — LOCKED FAILURE FOR PITCH RECOVERY"
        elif self.label == "YAW_ONLY_RECOVERY":
            heading = "SAME TARGET — YAW-ONLY RECOVERY SUCCESS"
        elif self.label == "PITCH_ONLY_RECOVERY":
            heading = "SAME TARGET — PITCH-ONLY RECOVERY SUCCESS"
        else:
            heading = self.label
        failure = self.row["failure_label"] or self.audit["cases"][self.label]["replay_class"]
        lines = [
            heading,
            f"target: ({self.row['target_x']}, {self.row['target_y']}, {self.row['target_z']}) m | Lift: {self.row['lift']} m",
            f"Yaw/Pitch: {yaw} / {pitch}",
            f"stage: {frame['stage']} | label: {failure}",
        ]
        for index, line in enumerate(lines):
            text = Marker(); text.header.frame_id="world"; text.header.stamp=self.get_clock().now().to_msg()
            text.ns="replay_text";text.id=100+index;text.type=Marker.TEXT_VIEW_FACING;text.action=Marker.ADD
            text.pose.position.x=float(self.scene["box"]["center_xyz"][0])
            text.pose.position.y=float(self.scene["box"]["center_xyz"][1])
            text.pose.position.z=1.27-index*0.045;text.pose.orientation.w=1.0
            text.scale.z=0.032 if index == 0 else 0.020;text.color=color(1,1,1,1)
            text.text=line;array.markers.append(text)
        return array

    def update_tcp_audit(self, frame):
        if self.failure_visual:
            return
        try:
            transform = self.tf_buffer.lookup_transform("world", TCP_LINK, rclpy.time.Time())
        except Exception:
            return
        t = transform.transform.translation; q = transform.transform.rotation
        current = ([t.x,t.y,t.z],[q.x,q.y,q.z,q.w])
        stamp=(transform.header.stamp.sec,transform.header.stamp.nanosec)
        matched_lift=self.lift_by_stamp.get(stamp)
        if matched_lift is None:
            return
        if self.reference_tcp is None:
            self.reference_tcp = (current, matched_lift)
            return
        reference, reference_lift = self.reference_tcp
        self.max_tcp_xy_error=max(self.max_tcp_xy_error,math.hypot(t.x-reference[0][0],t.y-reference[0][1]))
        expected_z=reference[0][2]-(matched_lift-reference_lift)
        self.max_tcp_z_error=max(self.max_tcp_z_error,abs(t.z-expected_z))
        dot=abs(sum(current[1][i]*reference[1][i] for i in range(4)));dot=min(1.0,max(-1.0,dot))
        self.max_tcp_orientation_error=max(self.max_tcp_orientation_error,2*math.acos(dot))

    def write_runtime(self, state):
        payload={"protocol":"PAPER_RESULT_RVIZ_REPLAY_V1","state":state,"case_label":self.label,
                 "unique_key":self.row["unique_key"],"planning_reexecuted":False,"ik_reexecuted":False,
                 "robot_model_joint_names_match":True,"lift_axis_world":[0.0,0.0,-1.0],
                 "displayed_saved_joint_max_error":0.0 if not self.failure_visual else None,
                 "arm_max_change_during_lift":0.0 if not self.failure_visual else None,
                 "lift_grasp":float(self.row["lift"]),
                 "lift_approach_and_clearance":float(self.row["lift"])-0.17,
                 "lift_descent_direction":"INCREASING_Q_DOWNWARD",
                 "lift_ascent_direction":"DECREASING_Q_UPWARD",
                 "expected_object_box_top_clearance":0.02 if not self.failure_visual else None,
                 "max_tcp_xy_error":self.max_tcp_xy_error if self.reference_tcp else None,
                 "max_tcp_z_error":self.max_tcp_z_error if self.reference_tcp else None,
                 "max_tcp_orientation_error":self.max_tcp_orientation_error if self.reference_tcp else None,
                 "visual_penetration_review":"RVIZ_USER_VISIBLE",
                 "failure_label":self.row["failure_label"] or None,
                 "failure_collision_pair":self.row["collision_pairs"] or None}
        tmp=self.runtime_path.with_suffix(self.runtime_path.suffix+".tmp");tmp.write_text(yaml.safe_dump(payload,sort_keys=False));os.replace(tmp,self.runtime_path)

    def tick(self):
        if self.failure_visual:
            frame=self.frames[0]
        elif self.frame_index < len(self.frames):
            frame=self.frames[self.frame_index];self.frame_index += 1
        else:
            frame={"stage":"CLEARANCE_HOLD","lift":float(self.row["lift"])-0.17,"attached":True,
                   "finger":float(self.scene["task"]["q_contact_50mm"])}
        joints=self.joint_state(frame)
        if not self.failure_visual:self.joint_pub.publish(joints)
        self.publish_scene(frame,joints);self.marker_pub.publish(self.markers(frame));self.update_tcp_audit(frame)
        if self.failure_visual:
            self.write_runtime("DISPLAYING_STATIC_FAILURE")
            return
        if self.frame_index % 20 == 0 or frame["stage"] == "CLEARANCE_HOLD":
            self.write_runtime("HOLDING_FINAL_CLEARANCE" if frame["stage"] == "CLEARANCE_HOLD" else "REPLAYING")


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--result-root",required=True);parser.add_argument("--selection-audit",required=True)
    parser.add_argument("--case-label",default="LOCKED_COMMON_SUCCESS",choices=LABELS)
    parser.add_argument("--scene-config",required=True);parser.add_argument("--robot-xacro",required=True)
    parser.add_argument("--runtime-audit",required=True);parser.add_argument("--playback-speed",type=float,default=0.5)
    args,ros_args=parser.parse_known_args();rclpy.init(args=ros_args);node=None
    try:
        node=Replay(args);rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__ == "__main__":
    main()

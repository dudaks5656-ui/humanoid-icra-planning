#!/usr/bin/env python3
import csv,rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker,MarkerArray
WS="/home/openarm/humanoid_sim_ws";V=f"{WS}/validation";CONFIGS=["LIFT_ONLY","LIFT_YAW","LIFT_PITCH","LIFT_YAW_PITCH"];COLORS=[(.18,.57,1),(1,.62,.11),(.87,.39,.86),(.15,.83,.76)]
def read(n):
 with open(f"{V}/{n}",newline="",encoding="utf-8") as f:return list(csv.DictReader(f))
def pt(r,prefix="tcp"):p=Point();p.x=float(r[f"{prefix}_x"]);p.y=float(r[f"{prefix}_y"]);p.z=float(r[f"{prefix}_z"]);return p
class Demo(Node):
 def __init__(self):
  super().__init__("boundary_sweep_workspace_rviz");self.declare_parameter("scene","boundary_compare");self.declare_parameter("pose_type","MAX_REACH_POSE");self.scene=self.get_parameter("scene").value;self.pose_type=self.get_parameter("pose_type").value
  self.nested=read("boundary_sweep_workspace_nested_states.csv");self.surface=read("boundary_sweep_workspace_3d_surface.csv");self.poses=read("boundary_sweep_workspace_representative_poses.csv");self.mp=self.create_publisher(MarkerArray,"/boundary_sweep_workspace/markers",1);self.jp=self.create_publisher(JointState,"/joint_states",1);self.create_timer(.5,self.publish)
 def active(self):return CONFIGS if self.scene=="boundary_compare" else [{f"boundary_c{i}":c for i,c in enumerate(CONFIGS)}.get(self.scene,"LIFT_YAW_PITCH")]
 def publish(self):
  arr=MarkerArray();mid=0
  for c in self.active():
   color=COLORS[CONFIGS.index(c)]
   for b,width,alpha in [("OUTER_BOUNDARY",.012,.95),("INNER_BOUNDARY",.008,.75)]:
    for view in ("FRONT","RIGHT"):
     for lift in sorted({float(r["lift_value"]) for r in self.nested}):
      rs=sorted([r for r in self.nested if r["target_configuration"]==c and r["view"]==view and r["boundary_type"]==b and abs(float(r["lift_value"])-lift)<1e-7],key=lambda r:float(r["sweep_parameter"]));m=Marker();m.header.frame_id="base_link";m.header.stamp=self.get_clock().now().to_msg();m.ns=f"{c}_{view}_{b}_{lift}";m.id=mid;mid+=1;m.type=Marker.LINE_STRIP;m.action=Marker.ADD;m.scale.x=width;m.color.r,m.color.g,m.color.b=color;m.color.a=alpha;m.points=[pt(r) for r in rs]+([pt(rs[0])] if rs else []);arr.markers.append(m)
   m=Marker();m.header.frame_id="base_link";m.header.stamp=self.get_clock().now().to_msg();m.ns=f"{c}_shell";m.id=mid;mid+=1;m.type=Marker.TRIANGLE_LIST;m.action=Marker.ADD;m.color.r,m.color.g,m.color.b=color;m.color.a=.12
   for r in [x for x in self.surface if x["configuration"]==c]:
    for prefix in ("v1","v2","v3"):m.points.append(pt(r,prefix))
   arr.markers.append(m)
  self.mp.publish(arr);c=self.active()[-1];pose=next((r for r in self.poses if r["configuration"]==c and r["view"]=="RIGHT" and r["pose_type"]==self.pose_type),None)
  if pose:msg=JointState();msg.header.stamp=self.get_clock().now().to_msg();msg.name=pose["joint_names"].split(";");msg.position=list(map(float,pose["joint_values"].split(";")));self.jp.publish(msg)
def main():
 rclpy.init();n=Demo()
 try:rclpy.spin(n)
 except (ExternalShutdownException,KeyboardInterrupt):pass
 finally:
  n.destroy_node()
  if rclpy.ok():rclpy.shutdown()
if __name__=="__main__":main()

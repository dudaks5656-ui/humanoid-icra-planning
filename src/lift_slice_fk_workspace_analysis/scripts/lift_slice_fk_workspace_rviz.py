#!/usr/bin/env python3
"""RViz-only lift-slice endpoint, loft surface, and representative-state publisher."""
import csv, os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

WS="/home/openarm/humanoid_sim_ws"; VAL=f"{WS}/validation"
CONFIGS=["LIFT_ONLY","LIFT_YAW","LIFT_PITCH","LIFT_YAW_PITCH"]
COLORS=[(0.18,.57,1.),(1.,.62,.11),(.87,.39,.86),(.15,.83,.76)]

def rows(path):
    with open(path,newline="",encoding="utf-8") as f:return list(csv.DictReader(f))
def point(x,y,z): p=Point();p.x=float(x);p.y=float(y);p.z=float(z);return p

class Publisher(Node):
    def __init__(self):
        super().__init__("lift_slice_fk_workspace_rviz")
        self.declare_parameter("scene","lift_compare");self.declare_parameter("pose_slice","mid")
        self.scene=self.get_parameter("scene").value;self.pose_slice=self.get_parameter("pose_slice").value.upper()
        self.marker_pub=self.create_publisher(MarkerArray,"/lift_slice_fk_workspace/markers",1)
        self.state_pub=self.create_publisher(JointState,"/joint_states",1)
        self.points=rows(f"{VAL}/lift_slice_fk_workspace_nested_points.csv")
        self.tris=rows(f"{VAL}/lift_slice_fk_workspace_3d_surface.csv")
        self.poses=rows(f"{VAL}/lift_slice_fk_workspace_representative_states.csv")
        self.timer=self.create_timer(.5,self.publish)

    def active(self):
        if self.scene=="lift_compare":return CONFIGS
        mapping={f"lift_c{i}":c for i,c in enumerate(CONFIGS)}
        return [mapping.get(self.scene,"LIFT_YAW_PITCH")]

    def publish(self):
        out=MarkerArray();mid=0
        for config in self.active():
            ci=CONFIGS.index(config);color=COLORS[ci]
            for ratio in (0.,.25,.5,.75,1.):
                m=Marker();m.header.frame_id="base_link";m.header.stamp=self.get_clock().now().to_msg();m.ns=f"{config}_{ratio}";m.id=mid;mid+=1
                m.type=Marker.POINTS;m.action=Marker.ADD;m.scale.x=.009;m.scale.y=.009
                m.color.r,m.color.g,m.color.b=color;m.color.a=.10+.12*ratio
                m.points=[point(r["tcp_x"],r["tcp_y"],r["tcp_z"]) for r in self.points if r["target_configuration"]==config and abs(float(r["lift_ratio"])-ratio)<1e-8][::3]
                out.markers.append(m)
            m=Marker();m.header.frame_id="base_link";m.header.stamp=self.get_clock().now().to_msg();m.ns=f"{config}_loft";m.id=mid;mid+=1
            m.type=Marker.TRIANGLE_LIST;m.action=Marker.ADD;m.color.r,m.color.g,m.color.b=color;m.color.a=.20
            selected=[r for r in self.tris if r["configuration"]==config]
            for r in selected:
                for prefix in ("v1","v2","v3"):m.points.append(point(r[f"{prefix}_x"],r[f"{prefix}_y"],r[f"{prefix}_z"]))
            out.markers.append(m)
        text=Marker();text.header.frame_id="base_link";text.header.stamp=self.get_clock().now().to_msg();text.ns="caption";text.id=mid
        text.type=Marker.TEXT_VIEW_FACING;text.action=Marker.ADD;text.pose.position.x=.2;text.pose.position.y=-.45;text.pose.position.z=1.9;text.scale.z=.07;text.color.r=text.color.g=text.color.b=1.;text.color.a=1.
        text.text="LIFT-SLICE POSITIONAL FK WORKSPACE\nAMR FIXED · VISUALIZATION ONLY\nNo IK / no controller / no execution";out.markers.append(text)
        self.marker_pub.publish(out)
        config=self.active()[-1];pose=next((r for r in self.poses if r["configuration"]==config and r["slice_label"]==self.pose_slice),None)
        if pose:
            msg=JointState();msg.header.stamp=self.get_clock().now().to_msg();msg.name=pose["joint_names"].split(";");msg.position=list(map(float,pose["joint_values"].split(";")));self.state_pub.publish(msg)

def main():
    rclpy.init();node=Publisher();rclpy.spin(node);node.destroy_node();rclpy.shutdown()
if __name__=="__main__":main()

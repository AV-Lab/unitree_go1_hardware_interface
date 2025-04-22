#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, PoseStamped
from visualization_msgs.msg import Marker
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
import moveit_commander
import tf.transformations as tft
from std_msgs.msg import Float64
from sdk_sagittarius_arm.msg import ArmRadControl
import yaml
import os
from std_msgs.msg import String


class MoveAndVisualizeObject:
    def __init__(self):
        rospy.init_node("move_to_object_node")
        rospy.sleep(5.0)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        moveit_commander.roscpp_initialize([])
        self.robot = RobotCommander(robot_description="/sgr532/robot_description")
        self.scene = PlanningSceneInterface(ns="/sgr532")
        self.group = MoveGroupCommander("sagittarius_arm", robot_description="/sgr532/robot_description", ns="/sgr532")
        self.group.set_pose_reference_frame("sgr532/base_link")
        self.group.set_planning_time(5)
        self.group.set_max_velocity_scaling_factor(0.3)
        self.group.set_max_acceleration_scaling_factor(0.3)
        self.detected_color = "green" 


        self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=1)
        self.gripper_pub = rospy.Publisher("/sgr532/gripper/command", Float64, queue_size=10)
        self.arm_pub = rospy.Publisher("/sgr532/joint/commands", ArmRadControl, queue_size=10)

        rospy.Subscriber("/object_position", PointStamped, self.callback)
        rospy.Subscriber("/object_size", PointStamped, self.size_callback)
        rospy.Subscriber("/object_color", String, self.color_callback)


        self.last_size = (0.06, 0.05, 0.06) 
        self.post_pick_pose = self.load_post_pick_pose()
        self.busy = False\

        rospy.loginfo("MoveToObject + Visualization node ready.")
        rospy.spin()

        # Wait until the gripper command topic has at least one subscriber
        while self.gripper_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.1)

        self.gripper_pub.publish(Float64(data=0))
        rospy.loginfo("✊ Gripper opened.")


    def load_post_pick_pose(self):
        config_path = rospy.get_param("~pose_config", "")
        if not os.path.isfile(config_path):
            rospy.logwarn("⚠️ Pose config not found, using default post-pick.")
            return [0.0, 0.3, 0.0, 0.4, -1.8, 0.0]

        with open(config_path, 'r') as f:
            data = yaml.safe_load(f)
            return data.get("post_pick_rad", [0.0, 0.3, 0.0, 0.4, -1.8, 0.0])

    def size_callback(self, msg):
        self.last_size = (msg.point.x, msg.point.x, msg.point.y)

    def color_callback(self, msg):
        self.detected_color = msg.data

    def callback(self, msg):
        if self.busy:
            return
        self.busy = True
        try:
            transform = self.tf_buffer.lookup_transform(
                "sgr532/base_link", msg.header.frame_id,
                rospy.Time(0), rospy.Duration(1.0)
            )
            transformed = tf2_geometry_msgs.do_transform_point(msg, transform)

            y_offset_correction = 0.030
            z_offset_correction = 0.025
            transformed.point.y += y_offset_correction
            transformed.point.z += z_offset_correction

            rospy.loginfo("Corrected object point: (%.3f, %.3f, %.3f)",
                          transformed.point.x, transformed.point.y, transformed.point.z)

            self.publish_marker(transformed)

            rospy.sleep(1.0)
            self.go_to_xyz(transformed.point.x, transformed.point.y, transformed.point.z)

            self.gripper_pub.publish(Float64(data=0.5))
            rospy.loginfo("✊ Gripper closed.")

            rospy.sleep(5.0)
            return_pose = ArmRadControl()
            return_pose.rad = self.post_pick_pose 
            self.arm_pub.publish(return_pose)
            rospy.loginfo("↩️ Returned to pre-pick pose.")
            rospy.sleep(5.0)
            self.busy = False   

        except Exception as e:
            rospy.logwarn("TF Transform failed: %s", str(e))

    def go_to_xyz(self, x, y, z):
        pose = PoseStamped()
        pose.header.frame_id = "sgr532/base_link"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        quat = tft.quaternion_from_euler(0, 1.57, 0)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        self.group.set_pose_target(pose)
        success = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

        if success:
            rospy.loginfo("Move executed.")
        else:
            rospy.logwarn("Planning failed.")

    def publish_marker(self, point_msg):
        box_length, box_width, box_height = self.last_size

        marker = Marker()
        marker.header.frame_id = point_msg.header.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "detected_object"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = point_msg.point.x
        marker.pose.position.y = point_msg.point.y
        marker.pose.position.z = point_msg.point.z - (box_height / 2.0)

        marker.pose.orientation.w = 1.0
        marker.scale.x = box_length
        marker.scale.y = box_width
        marker.scale.z = box_height
        color_map = {
            "red": (1.0, 0.0, 0.0),
            "green": (0.0, 1.0, 0.0),
            "blue": (0.0, 0.0, 1.0),
            "yellow": (1.0, 1.0, 0.0),
            "orange": (1.0, 0.5, 0.0),
            "purple": (0.5, 0.0, 0.5)
        }
        r, g, b = color_map.get(self.detected_color.lower(), (0.0, 1.0, 0.0))
        marker.color.a = 1.0
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b


        self.marker_pub.publish(marker)

if __name__ == '__main__':
    MoveAndVisualizeObject()

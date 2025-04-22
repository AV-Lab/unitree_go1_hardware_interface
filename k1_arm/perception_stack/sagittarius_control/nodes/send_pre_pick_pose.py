#!/usr/bin/env python3

import rospy
import yaml
from sdk_sagittarius_arm.msg import ArmRadControl

def load_joint_config():
    config_path = rospy.get_param("~config_path", "")
    if not config_path:
        rospy.logerr("No config path provided for pre-pick pose.")
        return None

    with open(config_path, 'r') as f:
        data = yaml.safe_load(f)
    return data.get("pre_pick_rad", None)

def main():
    rospy.init_node("pickup_pose_sender", anonymous=True)

    pub = rospy.Publisher('/sgr532/joint/commands', ArmRadControl, queue_size=10)

    # 🔁 Load joint config
    joints = load_joint_config()
    if joints is None:
        rospy.logerr("Failed to load joint angles. Exiting.")
        return

    rospy.loginfo("Waiting for subscribers on /sgr532/joint/commands...")
    while pub.get_num_connections() == 0 and not rospy.is_shutdown():
        rospy.sleep(0.1)

    msg = ArmRadControl()
    msg.rad = joints

    rospy.loginfo(f"Sending pre-pick joint positions: {joints}")
    for _ in range(5):
        pub.publish(msg)
        rospy.sleep(0.1)
    rospy.loginfo("Sent!")

if __name__ == '__main__':
    main()

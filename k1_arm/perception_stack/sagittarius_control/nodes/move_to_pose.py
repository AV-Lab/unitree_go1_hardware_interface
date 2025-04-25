#!/usr/bin/env python3

import rospy
import sys
from sdk_sagittarius_arm.msg import ArmRadControl

def main():
    rospy.init_node("move_to_pose_node", anonymous=True)

    if len(sys.argv) != 7:  # Expect 6 joint angles
        rospy.logerr("Usage: rosrun <your_pkg> pose_runner.py J1 J2 J3 J4 J5 J6")
        sys.exit(1)

    try:
        joints = [float(j) for j in sys.argv[1:]]
    except ValueError:
        rospy.logerr("All joint arguments must be valid numbers.")
        sys.exit(1)

    pub = rospy.Publisher("/sgr532/joint/commands", ArmRadControl, queue_size=10)
    rospy.sleep(1.0)  # Allow publisher to initialize

    # Step 1: Send initial pose
    msg = ArmRadControl()
    msg.rad = joints
    rospy.loginfo(f"➡️ Sending target joint positions: {joints}")
    pub.publish(msg)

    rospy.sleep(3.0)  # Hold the pose

    # Step 2: Return to default pose
    return_pose = ArmRadControl()
    return_pose.rad = [0.0, 0.3, 0.0, 0.4, -1.8, 0.0]
    rospy.loginfo("↩️ Returning to default pose...")
    pub.publish(return_pose)

if __name__ == "__main__":
    main()

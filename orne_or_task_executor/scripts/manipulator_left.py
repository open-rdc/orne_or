#!/usr/bin/env python3
# coding: UTF-8

import sys
from math import pi
import geometry_msgs
import moveit_commander
import rospy
import tf
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3

def callback(tf_object_position, args):
    left_arm = args[0]
    try:
        left_arm_goal = PoseStamped()
        left_arm_goal.header.frame_id = "world"  # Or the frame_id from tf_object_position if different
        left_arm_goal.pose.position.x = tf_object_position.position.x
        left_arm_goal.pose.position.y = tf_object_position.position.y
        left_arm_goal.pose.position.z = tf_object_position.position.z
        left_arm_goal.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, pi))
        left_arm.set_pose_target(left_arm_goal)
        left_arm.go(wait=True)
    except rospy.ROSInterruptException:
        return

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("left_manipulator_pose_and_hand_control")

    left_arm = moveit_commander.MoveGroupCommander("left_arm")
    left_hand = moveit_commander.MoveGroupCommander("left_hand")

    joint_goal = left_hand.get_current_joint_values()
    joint_goal[left_hand.get_active_joints().index('l_hand_left_joint')] = -pi / 4
    left_hand.go(joint_goal, wait=True)

    rospy.Subscriber("/tf_object_position", PoseStamped, callback, (left_arm,))

    rospy.spin()

if __name__ == "__main__":
    main()


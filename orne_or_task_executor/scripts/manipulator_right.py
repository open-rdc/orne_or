#!/usr/bin/env python3
# coding:UTF-8

import sys
from math import pi
import geometry_msgs
import moveit_commander
import rospy
import tf
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("right_manipulator_pose_and_hand_control")

    right_arm = moveit_commander.MoveGroupCommander("right_arm")
    right_hand = moveit_commander.MoveGroupCommander("right_hand")

    right_wrist_goal = PoseStamped()
    right_wrist_goal.header.frame_id = "world"
    right_wrist_goal.pose.position.x = 0.4517
    right_wrist_goal.pose.position.y = 0.0318
    right_wrist_goal.pose.position.z = 0.568653
    right_wrist_goal.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, -(pi / 2), pi))
    
    right_arm.set_pose_target(right_wrist_goal)

    right_arm.go(wait=True)

    target_joint_positions = [10 * (pi / 180)]
    right_hand.set_joint_value_target(target_joint_positions)
    right_hand.go(wait=True)
    
if __name__ == "__main__":
    main()

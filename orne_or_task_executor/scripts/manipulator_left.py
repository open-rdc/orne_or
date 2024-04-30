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
    rospy.init_node("left_manipulator_pose_and_hand_control")

    left_arm = moveit_commander.MoveGroupCommander("left_arm")
    left_hand = moveit_commander.MoveGroupCommander("left_hand")

    joint_goal = left_hand.get_current_joint_values()
    joint_goal[left_hand.get_active_joints().index('l_hand_left_joint')] = -pi / 4
    left_hand.go(joint_goal, wait=True)

    left_arm_goal = PoseStamped()
    left_arm_goal.header.frame_id = "world"
    left_arm_goal.pose.position.x = 0.686265
    left_arm_goal.pose.position.y = 0.406123
    left_arm_goal.pose.position.z = 0.796995
    left_arm_goal.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, pi))
    
    left_arm.set_pose_target(left_arm_goal)

    left_arm.go(wait=True)

    #target_joint_positions = [10 * (pi / 180)]
    #right_hand.set_joint_value_target(target_joint_positions)
    #right_hand.go(wait=True)
    
if __name__ == "__main__":
    main()

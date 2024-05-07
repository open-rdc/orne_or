#!/usr/bin/env python3
# coding: UTF-8

import sys
from math import pi
import rospy
import moveit_commander

def main():
    # ROSの初期化
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("dual_manipulator_pose_and_hand_control")

    right_arm = moveit_commander.MoveGroupCommander("right_arm")
    right_hand = moveit_commander.MoveGroupCommander("right_hand")
    left_arm = moveit_commander.MoveGroupCommander("left_arm")
    left_hand = moveit_commander.MoveGroupCommander("left_hand")
    #left_hand
    joint_goal = left_hand.get_current_joint_values()
    joint_goal[left_hand.get_active_joints().index('l_hand_left_joint')] = 0

    left_hand.go(joint_goal, wait=True)

    left_hand.stop()
    left_hand.clear_pose_targets()
    #right_arm
    right_joint_goal = {
        'r_sholder_roll_joint': -pi / 2.001,
        'r_elbow_roll_joint': pi * 16.95 / 18
    }
    right_arm.set_joint_value_target(right_joint_goal)
    right_arm.go(wait=True)
    #left_arm
    left_joint_goal = {
        'l_sholder_roll_joint': -pi / 2.001,
        'l_elbow_roll_joint': pi * 16.95 / 18
    }
    left_arm.set_joint_value_target(left_joint_goal)
    left_arm.go(wait=True)

    left_arm.stop()
    left_arm.clear_pose_targets()
    #right_arm
    right_joint_goal = {
        'r_sholder_roll_joint': pi / 5,
        'r_elbow_roll_joint': pi / 3
    }
    right_arm.set_joint_value_target(right_joint_goal)
    right_arm.go(wait=True)
    #right_hand
    joint_goal = right_hand.get_current_joint_values()
    joint_goal[right_hand.get_active_joints().index('r_hand_joint')] = pi / 11.25

    right_hand.go(joint_goal, wait=True)
    right_hand.stop()
    right_hand.clear_pose_targets()
    #right_arm
    right_joint_goal = {
        'r_sholder_roll_joint': -pi / 2.001,
        'r_elbow_roll_joint': pi * 16.95 / 18
    }
    right_arm.set_joint_value_target(right_joint_goal)
    right_arm.go(wait=True)
    
    right_arm.stop()
    right_arm.clear_pose_targets()

if __name__ == "__main__":
    main()

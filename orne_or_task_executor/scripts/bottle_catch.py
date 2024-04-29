#!/usr/bin/env python3
# coding: UTF-8

import sys
from math import pi
import geometry_msgs.msg
import moveit_commander
import rospy
import tf
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3

def main():
    # ROSの初期化
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("dual_manipulator_pose_and_hand_control")

    right_arm = moveit_commander.MoveGroupCommander("right_arm")
    left_arm = moveit_commander.MoveGroupCommander("left_arm")
    left_hand = moveit_commander.MoveGroupCommander("left_hand")
    # ホームポジション
    right_joint_goal = {
        'r_sholder_roll_joint': -pi / 2.001,
        'r_sholder_pitch_joint': 0,
        'r_elbow_roll_joint': pi * 16.95 / 18,
        'r_wrist_roll_joint': 0,
        'r_wrist_pitch_joint': 0,
        'r_wrist_yaw_joint': 0
    }
    right_arm.set_joint_value_target(right_joint_goal)
    right_arm.go(wait=True)
    left_joint_goal = {
        'l_sholder_roll_joint': -pi / 2.001,
        'l_sholder_pitch_joint': 0,
        'l_elbow_roll_joint': pi * 16.95 / 18,
        'l_wrist_roll_joint': 0,
        'l_wrist_pitch_joint': 0,
        'l_wrist_yaw_joint': 0
    }
    left_arm.set_joint_value_target(left_joint_goal)
    left_arm.go(wait=True)
    # ハンドを開く
    joint_goal = left_hand.get_current_joint_values()
    joint_goal[left_hand.get_active_joints().index('l_hand_left_joint')] = -pi / 4
    # 目標値に移動
    left_hand.go(joint_goal, wait=True)
    left_joint_goal = {
        'l_sholder_roll_joint': pi / 4.1,
        'l_elbow_roll_joint': pi / 3.2,
        'l_wrist_roll_joint': pi / 3.6
    }
    left_arm.set_joint_value_target(left_joint_goal)
    left_arm.go(wait=True)
    # ハンドを閉じる
    joint_goal = left_hand.get_current_joint_values()
    joint_goal[left_hand.get_active_joints().index('l_hand_left_joint')] = 0
    left_hand.go(joint_goal, wait=True)
    # ホームポジション
    left_joint_goal = {
        'l_sholder_roll_joint': -pi / 2.001,
        'l_sholder_pitch_joint': 0,
        'l_elbow_roll_joint': pi * 16.95 / 18,
        'l_wrist_roll_joint': 0,
        'l_wrist_pitch_joint': 0,
        'l_wrist_yaw_joint': 0
    }
    left_arm.set_joint_value_target(left_joint_goal)
    left_arm.go(wait=True)

    left_hand.stop()
    left_hand.clear_pose_targets()

if __name__ == "__main__":
    main()

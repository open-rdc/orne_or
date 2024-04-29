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

    # 右アームと左アームの初期化
    right_arm = moveit_commander.MoveGroupCommander("right_arm")

    # 右腕のジョイント値を設定
    right_joint_goal = {
        'r_sholder_roll_joint': pi / 2.001
    }
    right_arm.set_joint_value_target(right_joint_goal)
    right_arm.go(wait=True)

    # アームを停止し、ターゲットをクリア
    right_arm.stop()
    right_arm.clear_pose_targets()

if __name__ == "__main__":
    main()


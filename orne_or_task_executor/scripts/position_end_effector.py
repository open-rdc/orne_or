#!/usr/bin/env python3
# coding: UTF-8

import sys
import rospy
import moveit_commander

def main():
    # ROSの初期化
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("get_end_effector_pose")

    # アームの初期化
    arm = moveit_commander.MoveGroupCommander("right_arm")

    # エンドエフェクタの現在のポーズを取得
    current_pose = arm.get_current_pose()
    print("Current pose of the end-effector:")
    print("Position:", current_pose.pose.position)
    print("Orientation:", current_pose.pose.orientation)

    # MoveIt!のシャットダウン
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()


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

    # 左右のアームの初期化
    right_arm = moveit_commander.MoveGroupCommander("right_arm")

    # 左右のアームを目標ポーズに移動
    right_arm.go(wait=True)

    # 右手の手首を回転
    r_wrist_yaw_joint_positions = [pi / 2]  # 正しいジョイント名を使用する
    right_arm.set_joint_value_target("r_wrist_yaw_joint", r_wrist_yaw_joint_positions)  # 正しいジョイント名を指定
    right_arm.go(wait=True)
    
if __name__ == "__main__":
    main()

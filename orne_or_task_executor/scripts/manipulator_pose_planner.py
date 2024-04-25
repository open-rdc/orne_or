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
    left_arm = moveit_commander.MoveGroupCommander("left_arm")
    right_arm = moveit_commander.MoveGroupCommander("right_arm")
    right_hand = moveit_commander.MoveGroupCommander("right_hand")

    # 左右のアームの目標ポーズの設定
    left_pose_goal = PoseStamped()
    left_pose_goal.header.frame_id = "world"
    left_pose_goal.pose.position = Vector3(0.23, 0.27, 0.54)
    left_pose_goal.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, pi))

    right_wrist_goal = PoseStamped()
    right_wrist_goal.header.frame_id = "world"
    right_wrist_goal.pose.position = Vector3(0.12, -0.26, 0.54)
    right_wrist_goal.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, pi))

    # 左右のアームに目標ポーズを設定
    left_arm.set_pose_target(left_pose_goal)
    right_arm.set_pose_target(right_wrist_goal)

    # 左右のアームを目標ポーズに移動
    left_arm.go(wait=True)
    right_arm.go(wait=True)

    # 右手の手首を回転
    r_wrist_yaw_joint_positions = [-20 * (pi / 180)]  # 正しいジョイント名を使用する
    right_arm.set_joint_value_target("r_wrist_yaw_joint", r_wrist_yaw_joint_positions)  # 正しいジョイント名を指定
    right_arm.go(wait=True)
    
    target_joint_positions = [10 * (pi / 180)]
    right_hand.set_joint_value_target(target_joint_positions)
    right_hand.go(wait=True)

    # 左手のハンドを開く
    left_arm_open_command = left_arm.get_current_joint_values()
    left_arm_open_command[-1] = 0.1  # 最後の関節がハンドを制御
    left_arm.set_joint_value_target(left_arm_open_command)
    left_arm.go(wait=True)

    # 右手のハンドを開く
    right_arm_open_command = right_arm.get_current_joint_values()
    right_arm_open_command[-1] = 0.1  # 最後の関節がハンドを制御
    right_arm.set_joint_value_target(right_arm_open_command)
    right_arm.go(wait=True)

    rospy.sleep(rospy.Duration(1.0))  # 1秒待つ

    # 左手のハンドを閉じる
    left_arm_close_command = left_arm.get_current_joint_values()
    left_arm_close_command[-1] = 0.0  # ハンドを閉じる
    left_arm.set_joint_value_target(left_arm_close_command)
    left_arm.go(wait=True)

    # 右手のハンドを閉じる
    right_arm_close_command = right_arm.get_current_joint_values()
    right_arm_close_command[-1] = 0.0  # ハンドを閉じる
    right_arm.set_joint_value_target(right_arm_close_command)
    right_arm.go(wait=True)

    # 左右のアームを停止し、ターゲットをクリア
    left_arm.stop()
    left_arm.clear_pose_targets()
    right_arm.stop()
    right_arm.clear_pose_targets()

if __name__ == "__main__":
    main()

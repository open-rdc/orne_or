#!/usr/bin/env python3
# coding: UTF-8

import rospy
import moveit_commander
from geometry_msgs.msg import Pose, Quaternion, Vector3
import time
import sys

def main():
    # ノードの初期化
    rospy.init_node("end_effector_controller")

    # MoveItの初期化
    moveit_commander.roscpp_initialize(sys.argv)

    # 左のアームのMoveGroupCommanderを作成
    move_group = moveit_commander.MoveGroupCommander("left_arm")

    # エンドエフェクターの最大速度を設定
    move_group.set_max_velocity_scaling_factor(0.5)

    # 目標座標のリスト
    target_positions = [
        Vector3(0.23, 0.27, 0.54),
        Vector3(0.3, 0.3, 0.5),
        Vector3(0.25, 0.25, 0.6)
    ]

    # 目標座標を順番に移動
    for target_position in target_positions:
        # 現在の姿勢を取得
        current_pose = move_group.get_current_pose().pose

        # 目標姿勢を設定
        pose_goal = Pose()
        pose_goal.position = target_position
        pose_goal.orientation = current_pose.orientation  # 姿勢を変更しない

        # 目標姿勢に移動
        move_group.set_pose_target(pose_goal)
        move_group.go(wait=True)

        # 移動が完了したら一時停止
        time.sleep(1)

    # MoveItの終了処理
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()

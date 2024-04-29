#!/usr/bin/env python3
# coding: UTF-8

import sys
from math import pi
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped, Quaternion, Point
import tf

# グローバル変数
arm = None

def point_callback(point_msg):
    """ポイントトピックから受け取った座標を使ってアームを動かす"""
    global arm
    if arm is not None:
        target_pose = PoseStamped()
        target_pose.header.frame_id = "world"
        target_pose.pose.position = point_msg  # 受け取ったPointを使用
        target_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, pi))

        # 目標ポーズを設定してアームを動かす
        arm.set_pose_target(target_pose)
        arm.go(wait=True)

def main():
    global arm

    # ROSの初期化
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("dual_manipulator_pose_and_hand_control")

    # 右アームの初期化
    arm = moveit_commander.MoveGroupCommander("right_arm")

    # 座標をパブリッシュするトピックをサブスクライブ
    rospy.Subscriber("point_topic", Point, point_callback)

    # ROSメッセージの処理ループを継続
    rospy.spin()

    # MoveItのシャットダウン
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()


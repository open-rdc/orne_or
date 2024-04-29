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
        try:
            # ジョイントの現在値を取得し、手首の回転を固定する
            wrist_yaw_angle = pi / 2  # 手首の回転を90度に保持
            joint_values = arm.get_current_joint_values()
            wrist_yaw_index = arm.get_active_joints().index('r_wrist_yaw_joint')
            joint_values[wrist_yaw_index] = wrist_yaw_angle

            # 手首のジョイント値を再設定して保持
            arm.set_joint_value_target(joint_values)
            arm.go(wait=True)

            # 目標ポーズを設定してアームを動かす
            target_pose = PoseStamped()
            target_pose.header.frame_id = "base_link"
            target_pose.pose.position = point_msg
            quat = tf.transformations.quaternion_from_euler(0, 0, pi)
            target_pose.pose.orientation.x = quat[0]
            target_pose.pose.orientation.y = quat[1]
            target_pose.pose.orientation.z = quat[2]
            target_pose.pose.orientation.w = quat[3]

            # 目標ポーズと手首のジョイント値を同時に設定
            arm.set_pose_target(target_pose)
            arm.set_joint_value_target(joint_values)  # この行を追加
            if not arm.go(wait=True):
                rospy.logerr("Failed to move arm to the desired pose with wrist fixed.")
        finally:
            arm.stop()
            arm.clear_pose_targets()

def main():
    global arm
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("dual_manipulator_pose_and_hand_control")
    arm = moveit_commander.MoveGroupCommander("right_arm")
    rospy.Subscriber("point_topic", Point, point_callback)
    rospy.spin()
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()


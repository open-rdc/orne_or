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
    moveit_commander.roscpp_initialize(sys.argv)

    rospy.init_node("dual_manipulator_pose_planner")

    left_arm = moveit_commander.MoveGroupCommander("left_arm")
    right_arm = moveit_commander.MoveGroupCommander("right_arm")

    # 左右のアームの目標姿勢を設定
    left_pose_goal = PoseStamped()
    left_pose_goal.header.frame_id = "world"
    left_pose_goal.pose.position = Vector3(0.23, 0.27, 0.54)
    left_pose_goal.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, pi))

    right_pose_goal = PoseStamped()
    right_pose_goal.header.frame_id = "world"
    right_pose_goal.pose.position = Vector3(0.12, -0.26, 0.54)
    right_pose_goal.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, pi))

    # 左右のアームに対して目標姿勢を設定
    left_arm.set_pose_target(left_pose_goal)
    right_arm.set_pose_target(right_pose_goal)

    # 左右のアームを同時に移動
    left_arm.go(wait=True)
    right_arm.go(wait=True)

    left_arm.stop()
    left_arm.clear_pose_targets()

    right_arm.stop()
    right_arm.clear_pose_targets()

if __name__ == "__main__":
    main()

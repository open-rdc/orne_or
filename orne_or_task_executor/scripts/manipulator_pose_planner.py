#!/usr/bin/env python3
# coding: UTF-8

import sys
from math import pi

import geometry_msgs.msg
from geometry_msgs.msg import Pose
import moveit_commander
import rospy
import tf
from geometry_msgs.msg import Quaternion, Vector3

def main():
    moveit_commander.roscpp_initialize(sys.argv)

    rospy.init_node("dual_manipulator_pose_planner")

    left_arm = moveit_commander.MoveGroupCommander("left_arm")
    right_arm = moveit_commander.MoveGroupCommander("right_arm")

    # 左右のアームの目標姿勢を設定
    left_pose_goal = geometry_msgs.msg.Pose()
    left_pose_goal.position = Vector3(0.23, 0.27, 0.54)
    left_q = tf.transformations.quaternion_from_euler(pi, 0, 0)
    left_pose_goal.orientation = Quaternion(x=left_q[0], y=left_q[1], z=left_q[2], w=left_q[3])

    right_pose_goal = geometry_msgs.msg.Pose()
    right_pose_goal.position = Vector3(0.12, -0.26, 0.54)
    right_q = tf.transformations.quaternion_from_euler(pi, 0, 0)
    right_pose_goal.orientation = Quaternion(x=right_q[0], y=right_q[1], z=right_q[2], w=right_q[3])

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

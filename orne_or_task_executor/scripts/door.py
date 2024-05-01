#!/usr/bin/env python3
# coding:UTF-8

import sys
from math import pi
import geometry_msgs
import moveit_commander
import rospy
import tf
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("right_manipulator_pose_and_hand_control")

    right_arm = moveit_commander.MoveGroupCommander("right_arm")
    right_hand = moveit_commander.MoveGroupCommander("right_hand")
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

    #ドアノブ上
    right_wrist_goal = PoseStamped()
    right_wrist_goal.header.frame_id = "world"
    right_wrist_goal.pose.position.x = 0.45
    right_wrist_goal.pose.position.y = 0.11
    right_wrist_goal.pose.position.z = 0.9
    right_wrist_goal.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, pi/2))
    
    right_arm.set_pose_target(right_wrist_goal)

    right_arm.go(wait=True)

    target_joint_positions = [10 * (pi / 180)]
    right_hand.set_joint_value_target(target_joint_positions)
    right_hand.go(wait=True)

    #ドアノブつかむ

    right_wrist_goal = PoseStamped()
    right_wrist_goal.header.frame_id = "world"
    right_wrist_goal.pose.position.x = 0.45
    right_wrist_goal.pose.position.y = 0.11
    right_wrist_goal.pose.position.z = 0.8
    right_wrist_goal.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, pi/2))
    
    right_arm.set_pose_target(right_wrist_goal)

    right_arm.go(wait=True)

    target_joint_positions = [10 * (pi / 180)]
    right_hand.set_joint_value_target(target_joint_positions)
    right_hand.go(wait=True)
    
    #ドアノブ引く
    right_wrist_goal = PoseStamped()
    right_wrist_goal.header.frame_id = "world"
    right_wrist_goal.pose.position.x = 0.3
    right_wrist_goal.pose.position.y = -0.2
    right_wrist_goal.pose.position.z = 0.8
    right_wrist_goal.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, pi/2))
    
    right_arm.set_pose_target(right_wrist_goal)

    right_arm.go(wait=True)

    target_joint_positions = [10 * (pi / 180)]
    right_hand.set_joint_value_target(target_joint_positions)
    right_hand.go(wait=True)
if __name__ == "__main__":
    main()

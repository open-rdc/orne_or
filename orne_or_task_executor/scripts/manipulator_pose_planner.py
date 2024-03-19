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

    rospy.init_node("/tf_object_position")
    rospy.init_node("/manipulator_pose_planner")

    move_group = moveit_commander.MoveGroupCommander("left_arm")
    pose_goal_sub = rospy.Subscriber("/tf_object_position", PointStamped, callback)
    print(pose_goal_sub)

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position = Vector3(pose_goal_sub)
    # pose_goal.position = Vector3(0.23, 0.27, 0.0)
    q = tf.transformations.quaternion_from_euler(pi, 0, 0)
    pose_goal.orientation = Quaternion(x= q[0], y=q[1], z=q[2], w=q[3])
    move_group.set_pose_target(pose_goal)

    move_group.go(wait=True)

    move_group.stop()
    move_group.clear_pose_targets()

if __name__ == "__main__":
    main()
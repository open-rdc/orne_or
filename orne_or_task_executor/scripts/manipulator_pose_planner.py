#!/usr/bin/env python3
# coding: UTF-8

import sys
import rospy
from geometry_msgs.msg import PointStamped, Pose, Quaternion
import moveit_commander
import tf
from math import pi

class ArmMover:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_arm_to_point')

        self.arm = moveit_commander.MoveGroupCommander("right_arm")
        self.subscriber = rospy.Subscriber('/tf_object_position', PointStamped, self.callback)
    
    def callback(self, msg):
        # 座標に移動する
        target_pose = Pose()
        target_pose.position.x = msg.point.x
        target_pose.position.y = msg.point.y
        target_pose.position.z = msg.point.z
        
        # オリエンテーションは事前に定義されたものを使用 (ここではZ軸周りに90度回転)
        orientation = tf.transformations.quaternion_from_euler(0, 0, pi/2)
        target_pose.orientation = Quaternion(*orientation)

        # ポーズ目標を設定
        self.arm.set_pose_target(target_pose)

        # プランと実行
        self.arm.go(wait=True)

        # ジョイントの調整
        joint_goal = self.arm.get_current_joint_values()
        joint_index = self.arm.get_active_joints().index('r_wrist_yaw_joint')
        joint_goal[joint_index] = pi / 2  # r_wrist_yaw_joint を90度回転
        self.arm.set_joint_value_target(joint_goal)

        # ジョイントのプランと実行
        self.arm.go(wait=True)

        # 動作後にすべてのターゲットをクリア
        self.arm.stop()
        self.arm.clear_pose_targets()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    mover = ArmMover()
    mover.run()


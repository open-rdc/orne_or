#!/usr/bin/env python

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
from tf.transformation import quaternion_from_euler

def main():
    rospy.init_node("arm_control")
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scalling_factor(0.1)
    arm.set_max_acceleration_scalling_factor(0.1)
    gripper = moveit_commander.MoveGroupCommander("gripper")

    while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
        rospy.sleep(1.0)
    rospy.sleep(1.0)

    print("Group names:")
    print(robot.get_group_names())

    print ("Current")


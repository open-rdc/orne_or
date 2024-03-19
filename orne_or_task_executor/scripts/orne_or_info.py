import sys

import moveit_commander
import rospy

def main():
    moveit_commander.roscpp_initialize(sys.argv)

    rospy.init_node("orne_or_info")

    robot = moveit_commander.RobotCommander()

    print("==Robot Info==")
    print("[name]", robot.get_name() )
    print("[planning_frame]", robot.get_planning_frame())
    print("[interface_description]", robot.get_interface_description())
    print("")

    print("==Joint Info==")
    print("[active_joints]", robot.get_active_joints())
    print("[joints]", robot.get_joints())
    print("[current_joint_values]", robot.get_current_joint_values())
    print("")

    print("==EndEffector Info==")
    print("[has_end_effector_link]", robot.has_end_effector_link())
    print("[end_effector_link]", robot.get_end_effector_link())
    print("[current_pose]", robot.get_current_pose())
    print("[current_rpy]", robot.get_current_rpy())
    print("")

if __name__ == "__main__":
    main()
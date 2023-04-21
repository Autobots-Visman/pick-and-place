#!/usr/bin/env python3
"""Move the arm to a gazebo model.

https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html
"""
import sys
from argparse import ArgumentParser

import cv2
import geometry_msgs.msg
import moveit_commander
import numpy as np
import rospy
from gazebo_msgs.srv import GetModelState, GetWorldProperties


def parse_args():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("name", type=str, help="name prefix of the model")
    parser.add_argument("--group-name", type=str, default="arm")
    # ignore any other args
    args, _ = parser.parse_known_args()
    return args


def main():
    rospy.init_node("moveit_gazebo_model", anonymous=True)

    rospy.wait_for_service("/gazebo/get_world_properties")
    rospy.wait_for_service("/gazebo/get_model_state")
    get_world_properties = rospy.ServiceProxy(
        "/gazebo/get_world_properties", GetWorldProperties
    )
    get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

    args = parse_args()

    # fetch the model state
    world_properties = get_world_properties()
    model_state = None
    model_name = None
    for model_name in world_properties.model_names:
        if not model_name.startswith(args.name):
            continue
        model_state = get_model_state(model_name, "")
        break
    if model_state is None:
        rospy.logwarn("No model found")
        return
    rospy.loginfo(f"Moving to model: {model_name}")
    rospy.loginfo(model_state)

    # now use moveit to move the arm to the model
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    move_group = moveit_commander.MoveGroupCommander(args.group_name)

    planning_frame = move_group.get_planning_frame()
    rospy.loginfo(f"planning frame: {planning_frame}")
    eef_link = move_group.get_end_effector_link()
    rospy.loginfo(f"end effector link: {eef_link}")
    group_names = robot.get_group_names()
    rospy.loginfo(f"groups: {group_names}")

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = model_state.pose.position.x
    pose_goal.position.y = model_state.pose.position.y
    pose_goal.position.z = model_state.pose.position.z - 1.0 + 0.1
    # pose_goal.orientation.x = model_state.pose.orientation.x
    # pose_goal.orientation.y = model_state.pose.orientation.y
    # pose_goal.orientation.z = model_state.pose.orientation.z
    # pose_goal.orientation.w = model_state.pose.orientation.w

    # from manually planned
    # pose_goal.position.x = 0.32854702046051304
    # pose_goal.position.y = 0.19751133391893996
    # pose_goal.position.z = 0.11877254126283932
    pose_goal.orientation.x = 0.00059729752987172
    pose_goal.orientation.y = -0.9997364036203059
    pose_goal.orientation.z = 0.0009579879734643368
    pose_goal.orientation.w = 0.022931392697358662

    # set the start state
    move_group.set_start_state_to_current_state()

    # parallel plan
    # set tolerance to 1cm
    move_group.set_goal_position_tolerance(0.01)
    move_group.set_goal_orientation_tolerance(0.05)
    move_group.set_num_planning_attempts(10)
    move_group.set_planning_time(5)

    # use collision-aware ik

    # set goal
    move_group.set_pose_target(pose_goal)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()


if __name__ == "__main__":
    main()

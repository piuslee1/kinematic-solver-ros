#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import json

class Main(object):
  """Main"""
  def __init__(self):
    super(Main, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)

    robot = moveit_commander.RobotCommander()


    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    planning_frame = group.get_planning_frame()
    print("============ Reference frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print("============ End effector: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print("============ Robot Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print("============ Printing robot state")
    print(robot.get_current_state())
    print("")
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

    rospy.Subscriber('arm_position', String, self.move_to_position)
    rospy.Subscriber('arm_difference', String, self.move_to_difference)
    rospy.spin()


  def move_to_position(self, position_message):

    position_dic = json.loads(position_message.data)
    pose_goal = self.group.get_current_pose().pose

    orientation = position_dic[u"orientation"]
    position = position_dic[u"position"]

    quaternion = quaternion_from_euler(orientation[u"x"], orientation[u"y"],orientation[u"z"])

    pose_goal.orientation.x = quaternion[0]
    pose_goal.orientation.y = quaternion[1]
    pose_goal.orientation.z = quaternion[2]
    pose_goal.orientation.w = quaternion[3]
    pose_goal.position.x = position[u"x"]
    pose_goal.position.y = position[u"y"]
    pose_goal.position.z = position[u"z"]

    self.go_to_pose_goal(pose_goal)

  def move_to_difference(self, position_message):
    position_dic = json.loads(position_message.data)

    group = self.group

    pose_goal = self.group.get_current_pose().pose

    orientation = position_dic[u"orientation"]
    position = position_dic[u"position"]

    quaternion = quaternion_from_euler(orientation[u"x"], orientation[u"y"],orientation[u"z"])

    pose_goal.orientation.x = pose_goal.orientation.x + quaternion[0]
    pose_goal.orientation.y = pose_goal.orientation.y + quaternion[1]
    pose_goal.orientation.z = pose_goal.orientation.z + quaternion[2]
    pose_goal.orientation.w = pose_goal.orientation.w + quaternion[3]
    pose_goal.position.x = pose_goal.position.x + position[u"x"]
    pose_goal.position.y = pose_goal.position.y + position[u"y"]
    pose_goal.position.z = pose_goal.position.z + position[u"z"]

    self.go_to_pose_goal(pose_goal)


  def go_to_pose_goal(self, pose_goal):
    group = self.group

    group.set_pose_target(pose_goal)

    plan = group.go(wait=True)

    group.stop()
    group.clear_pose_targets()


main = Main()
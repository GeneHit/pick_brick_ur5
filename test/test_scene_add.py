#!/usr/bin/env python
'''
  test the the move of UR5's end using compute_cartesian_path()
  replace the go_initial_pose.py in pick_brick_ur5 when using
'''
import rospy, sys, numpy as np
import moveit_commander
from copy import deepcopy
from geometry_msgs.msg import Twist
import moveit_msgs.msg
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from time import sleep
import time

from geometry_msgs.msg import PoseStamped

class scene_add_:
  def __init__(self):
    rospy.init_node("scene_add", anonymous=False)

    self.state_change_time = rospy.Time.now()

    rospy.loginfo("Starting node scene_add pick and place")


    # Initialize the move_group API
    moveit_commander.roscpp_initialize(sys.argv)

    self.scene = moveit_commander.PlanningSceneInterface()
    #robot = moveit_commander.RobotCommander()
    self.robot = moveit_commander.RobotCommander()
    # Initialize the move group for the ur5_arm
    self.arm = moveit_commander.MoveGroupCommander('manipulator')
    rospy.sleep(2)

    #rospy.on_shutdown(self.cleanup)

    # Get the name of the end-effector link
    self.end_effector_link = self.arm.get_end_effector_link()

    # Set the reference frame for pose targets
    reference_frame = "/base_link"

    # Set the ur5_arm reference frame accordingly
    self.arm.set_pose_reference_frame(reference_frame)

  def scene_add_groud(self):
    self.scene.remove_world_object('ground')
    ground_height = -0.1
    ground_size = [2.0, 2.0, 0.01]
    ground_pose = PoseStamped()
    ground_pose.header.frame_id = "/base_link"
    ground_pose.header.stamp = rospy.Time.now()
    ground_pose.pose.position.x = 0.0
    ground_pose.pose.position.y = 0.0
    ground_pose.pose.position.z = ground_height - ground_size[2] / 2.0
    ground_pose.pose.orientation.x = 0.0
    ground_pose.pose.orientation.y = 0.0
    ground_pose.pose.orientation.z = 0.0
    ground_pose.pose.orientation.w = 1.0
    self.scene.add_box('ground', ground_pose, ground_size)

  def scene_add_brick(self):
    box_name='brick'
    self.scene.remove_attached_object(self.end_effector_link, box_name)
    self.scene.remove_world_object(box_name)
    rospy.sleep(1)

    brick_size = [0.24, 0.115, 0.053]
    brick_pose = PoseStamped()
    brick_pose.header.frame_id = self.end_effector_link
    brick_pose.header.stamp = rospy.Time.now()
    brick_pose.pose.position.x = 0.0
    brick_pose.pose.position.y = 0.0
    brick_pose.pose.position.z = 0.054/2.0
    brick_pose.pose.orientation.x = 0.0
    brick_pose.pose.orientation.y = 0.0
    brick_pose.pose.orientation.z = 0.0
    brick_pose.pose.orientation.w = 1.0
    self.scene.add_box(box_name, brick_pose, brick_size)
    rospy.sleep(1)
    grasping_group = 'endeffector'
    #touch_links = self.robot.get_link_names(group=grasping_group)
    #touch_links = self.robot.get_link_names('manipulator')
    touch_links = self.end_effector_link
    self.scene.attach_box(self.end_effector_link, box_name, touch_links=touch_links)
    rospy.sleep(1)  



if __name__ == '__main__':
    scene=scene_add_()
    scene.scene_add_groud()
    scene.scene_add_brick()
    rospy.sleep(2)



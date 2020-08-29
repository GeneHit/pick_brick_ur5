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

from geometry_msgs.msg import PoseStamped,Pose

class go_initial_pose:
  def __init__(self):
    rospy.init_node("go_initial_pose", anonymous=False)

    self.state_change_time = rospy.Time.now()

    rospy.loginfo("Starting node moveit_cartesian_path")

    rospy.on_shutdown(self.cleanup)

    # Initialize the move_group API
    moveit_commander.roscpp_initialize(sys.argv)

    # Initialize the move group for the ur5_arm
    self.arm = moveit_commander.MoveGroupCommander('manipulator')

    # Get the name of the end-effector link
    self.end_effector_link = self.arm.get_end_effector_link()

    # Set the reference frame for pose targets
    reference_frame = "/base_link"

    # Set the ur5_arm reference frame accordingly
    self.arm.set_pose_reference_frame(reference_frame)

    # Allow replanning to increase the odds of a solution
    self.arm.allow_replanning(True)

    # Allow some leeway in position (meters) and orientation (radians)
    self.arm.set_goal_position_tolerance(0.01)
    self.arm.set_goal_orientation_tolerance(0.1)
    self.arm.set_planning_time(0.1)
    self.arm.set_max_acceleration_scaling_factor(.5)
    self.arm.set_max_velocity_scaling_factor(.5)

    # Get the current pose so we can add it as a waypoint
    start_pose = self.arm.get_current_pose(self.end_effector_link).pose
    #print type(start_pose)

    # Initialize the waypoints list
    self.waypoints= []
    self.pointx = []
    self.pointy = []
    # Set the first waypoint to be the starting pose
    # Append the pose to the waypoints list
    target_pose = PoseStamped()
    target_pose.header.frame_id = 'base_link'
    target_pose.pose.position.x = 0.10
    target_pose.pose.position.y = -0.2
    target_pose.pose.position.z = 0.33
    target_pose.pose.orientation.x = 1.0
    target_pose.pose.orientation.y = 0.0
    target_pose.pose.orientation.z = 0.0
    target_pose.pose.orientation.w = 0.0
    '''
    self.arm.set_pose_target(pose_goal,self.end_effector_link)
    # Set the internal state to the current state
    self.arm.set_start_state_to_current_state()
    #plan = self.arm.plan()
    #self.arm.execute(plan)
    self.arm.go()
    '''

    pose_goal = Pose()
    pose_goal.orientation.x = 1.0
    pose_goal.orientation.y = 0.0
    pose_goal.orientation.z = 0.0
    pose_goal.orientation.w = 0.0
    pose_goal.position.x = 0.1
    pose_goal.position.y = -0.2
    pose_goal.position.z = 0.33

    self.arm.set_pose_target(pose_goal)
    plan = self.arm.go(wait=True)
    print plan
    # Calling `stop()` ensures that there is no residual movement
    self.arm.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.arm.clear_pose_targets()

    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)

  def cleanup(self):
    rospy.loginfo("Stopping the robot")

    # Stop any current arm movement
    self.arm.stop()

    #Shut down MoveIt! cleanly
    rospy.loginfo("Shutting down Moveit!")
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)


if __name__ == '__main__':
  mp=go_initial_pose()


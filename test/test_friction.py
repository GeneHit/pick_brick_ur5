#!/usr/bin/env python
'''
  test the the move of UR5's end using compute_cartesian_path() up 30cm
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
    self.arm.set_goal_position_tolerance(0.001)
    self.arm.set_goal_orientation_tolerance(0.001)
    self.arm.set_planning_time(0.1)
    self.arm.set_max_acceleration_scaling_factor(.5)
    self.arm.set_max_velocity_scaling_factor(.5)

    # Get the current pose so we can add it as a waypoint
    start_pose = self.arm.get_current_pose(self.end_effector_link).pose

    # Initialize the waypoints list
    self.waypoints= []
    self.pointx = []
    self.pointy = []
    # Set the first waypoint to be the starting pose
    # Append the pose to the waypoints list
    wpose = deepcopy(start_pose)

    # Set the next waypoint to the right 0.5 meters

    #---initial position of vaccur_gripper---
    wpose.position.x = wpose.position.x-0.05
    #wpose.position.y = wpose.position.y-0.5271
    wpose.position.z = wpose.position.z+0.005

    #wpose.orientation.x = 0.9999
    #wpose.orientation.y = -0.0033
    #wpose.orientation.z = 0.0001
    #wpose.orientation.w = 0.0152

    self.waypoints.append(deepcopy(wpose))
    # Set the internal state to the current state
    # self.arm.set_pose_target(wpose)

    self.arm.set_start_state_to_current_state()

    # Plan the Cartesian path connecting the waypoints

    """moveit_commander.move_group.MoveGroupCommander.compute_cartesian_path(
            self, waypoints, eef_step, jump_threshold, avoid_collisios= True)

       Compute a sequence of waypoints that make the end-effector move in straight line segments that follow the
       poses specified as waypoints. Configurations are computed for every eef_step meters;
       The jump_threshold specifies the maximum distance in configuration space between consecutive points
       in the resultingpath. The return value is a tuple: a fraction of how much of the path was followed,
       the actual RobotTrajectory.

    """
    plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.01, 0.0, True)


    # plan = self.arm.plan()

    # If we have a complete plan, execute the trajectory
    if 1-fraction < 0.2:
        rospy.loginfo("Path computed successfully. Moving the arm.")
        num_pts = len(plan.joint_trajectory.points)
        rospy.loginfo("\n# intermediate waypoints = "+str(num_pts))
        self.arm.execute(plan)
        rospy.loginfo("Path execution complete.")
    else:
        rospy.loginfo("Path planning failed")

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


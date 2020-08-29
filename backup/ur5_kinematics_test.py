#!/usr/bin/env python
'''
  test the the move of UR5's end using joint controll
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
import tf
from ur5_kinematics import *

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

      self.current_joint_states = self.arm.get_current_joint_values()
      current_state=[0]*6
      for i in xrange(6):
        current_state[i] = self.current_joint_states[i]    

      orientationR=tf.transformations.quaternion_matrix([-0.5, 0.5, 0.5, 0.5])
      #orientationR=tf.transformations.quaternion_matrix(tf.transformations.quaternion_about_axis(1.5705,(0,1,0)))
      transT=[0.1052,-0.5271,0.2350]
    
      self.pose_states=self.get_pose_list(orientationR,transT)
      pose_state=[]
      pose_state.append(self.pose_states)

      self.ur5_kinematics=Kinematic()
      self.weights= [1.] * 6
      result_joint=self.ur5_kinematics.best_sol(self.weights,current_state, self.pose_states)
      self.arm.set_joint_value_target(result_joint)
      # Specify default (idle) joint states
      self.default_joint_states = self.arm.get_current_joint_values()
      self.default_joint_states[0] = -1.57691
      self.default_joint_states[1] = -1.71667
      self.default_joint_states[2] = 1.79266
      self.default_joint_states[3] = -1.67721
      self.default_joint_states[4] = -1.5705
      self.default_joint_states[5] = 1.5705*2

      #self.arm.set_joint_value_target(self.default_joint_states)

      # Set the internal state to the current state
      self.arm.set_start_state_to_current_state()
      plan = self.arm.plan()

      self.arm.execute(plan)

      #rospy.on_shutdown(self.cleanup)


    def cleanup(self):
      rospy.loginfo("Stopping the robot")

      # Stop any current arm movement
      self.arm.stop()

      #Shut down MoveIt! cleanly
      rospy.loginfo("Shutting down Moveit!")
      moveit_commander.roscpp_shutdown()
      moveit_commander.os._exit(0)

    def get_pose_list(self,R,T):
      pose_list=[0]*16
      print("pose_list")
      for i in xrange(3):
          for j in xrange(3):
            pose_list[i*4+j]=R[i][j]
            print(pose_list[i*4+j])
      pose_list[3]=T[0]
      pose_list[7]=T[1]
      pose_list[11]=T[2]
      pose_list[12]=0
      pose_list[13]=0
      pose_list[14]=0
      pose_list[15]=1
      return pose_list


if __name__ == '__main__':
  mp=go_initial_pose()


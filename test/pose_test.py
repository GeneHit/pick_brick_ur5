#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
  test the the move of UR5's end using compute_cartesian_path() to pick place
'''
import rospy, sys, numpy as np
import moveit_commander
from copy import deepcopy
from geometry_msgs.msg import Twist
import moveit_msgs.msg
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rospy_tutorials.msg import Floats
from std_msgs.msg import Bool,String
import math
from time import sleep
from pick_brick_ur5.srv import VacuumPump
#from ur5_notebook.msg import Tracker

from time import sleep

class brick_stack:
    def __init__(self):
        rospy.init_node("brick_stack", anonymous=False)

        self.pose_sub = rospy.Subscriber('brick_position', Floats,\
            self.tracking_callback, queue_size=1)

        #self.sysmode_pub = rospy.Publisher('sys_mode', String, queue_size=1)

        self.state_change_time = rospy.Time.now()

        rospy.loginfo("Starting brick_stack node moveit_cartesian_path")

        rospy.on_shutdown(self.cleanup)

        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
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

        # Specify default (idle) joint states
        self.intial_joint_states = self.arm.get_current_joint_values()
        self.intial_joint_states[0] = -1.57691
        self.intial_joint_states[1] = -1.71667
        self.intial_joint_states[2] = 1.79266
        self.intial_joint_states[3] = -1.67721
        self.intial_joint_states[4] = -1.5705
        self.intial_joint_states[5] = 1.5705*2

        #self.arm.set_joint_value_target(self.intial_joint_states)
        # Set the internal state to the current state
        #self.arm.set_start_state_to_current_state()
        #plan = self.arm.plan()
        #self.arm.execute(plan)
        
        # Get the current pose so we can add it as a waypoint
        self.initial_pose = self.arm.get_current_pose(self.end_effector_link).pose
        self.waypoints = [ ]
        #set the brick pose
        self.object = deepcopy(self.initial_pose)
        self.object.orientation.x = 1.0
        self.object.orientation.y = 0.00
        self.object.orientation.z = 0.00
        self.object.orientation.w = 0.00
        
        self.brick_flag=False

    def cleanup(self):
        rospy.loginfo("Stopping the robot")

        # Stop any current arm movement
        self.arm.stop()

        #Shut down MoveIt! cleanly
        rospy.loginfo("Shutting down Moveit!")
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    def tracking_callback(self, msg):
        #rospy.loginfo(msg)
        #print(msg)
        track_flag=int(msg.data[3])
        if track_flag == 1 :
            if np.isnan(msg.data[0]):
                pass
            else:
                self.object.position.x = msg.data[0]
                self.object.position.y = msg.data[1]
                self.object.position.z = msg.data[2]
                self.brick_flag=True
        else:
            self.brick_flag=False


def main(args):
    controller=brick_stack()
    controll_hz = rospy.Rate(15)
    while not rospy.is_shutdown():
        controller.waypoints=[]
        controller.waypoints.append(deepcopy(controller.object))

        # Set the internal state to the current state
        controller.arm.set_start_state_to_current_state()

        # Plan the Cartesian path connecting the waypoints
        plan, fraction = controller.arm.compute_cartesian_path(controller.waypoints, 0.01, 0.0, True)
        # If we have a complete plan, execute the trajectory
        if fraction-0.2 > 0.7:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            num_pts = len(plan.joint_trajectory.points)
            rospy.loginfo("\n# intermediate waypoints = "+str(num_pts))
            controller.arm.execute(plan)
            rospy.loginfo("Path execution complete.")
            break
        else:
            rospy.loginfo("Path planning failed\ncan't go to the brick")

        #controller.sysmode_pub.publish(str(sysmode))
        controll_hz.sleep()


    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down")

if __name__ == '__main__':
    main(sys.argv)


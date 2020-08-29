#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
  test the the move of UR5's end using compute_cartesian_path() to pick place
'''
import rospy, sys, numpy as np
import moveit_commander
from copy import deepcopy
from geometry_msgs.msg import Twist,PoseStamped
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

from gazebo_msgs.srv import GetWorldProperties,GetModelProperties,GetModelState
from time import sleep

class brick_stack:
    def __init__(self):
        rospy.init_node("brick_stack", anonymous=False)

        #self.sysmode_pub = rospy.Publisher('sys_mode', String, queue_size=1)

        self.state_change_time = rospy.Time.now()
        if False:
            rospy.loginfo("Starting brick_stack node moveit_cartesian_path")

            rospy.on_shutdown(self.cleanup)

            # Initialize the move_group API
            moveit_commander.roscpp_initialize(sys.argv)

            self.scene = moveit_commander.PlanningSceneInterface()
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

            self.arm.set_joint_value_target(self.intial_joint_states)
            # Set the internal state to the current state
            self.arm.set_start_state_to_current_state()
            plan = self.arm.plan()
            self.arm.execute(plan)

    def cleanup(self):
        rospy.loginfo("Stopping the robot")

        # Stop any current arm movement
        self.arm.stop()

        #Shut down MoveIt! cleanly
        rospy.loginfo("Shutting down Moveit!")
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


def main(args):
    controller=brick_stack()
    minx=0.1-0.05
    miny=-0.56-0.05
    minz=-0.04
    maxx=minx+0.1
    maxy=miny+0.1
    maxz=minz-0.1
    box_pose = PoseStamped()
    box_pose.header.frame_id = "vacuum_board"
    box_pose.pose.position.z = 0.1
    box_pose.pose.orientation.w = 1.0

    gazebo_world = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
    model_prop = rospy.ServiceProxy('/gazebo/get_model_properties', GetModelProperties)
    model_coordi = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    world = gazebo_world.call()
    print world
    red_brick = [ ]
    if world.success is True:
        for model in world.model_names:
            if model.startswith('red_bric'):
                red_brick.append(model)
                print red_brick
    for model in red_brick:
        model_p=model_prop.call(model)
        print model_p
        if model_p.success is True:   
            resp_coordi = model_coordi(model,'robot')
            print resp_coordi
            if resp_coordi.success is True:
                print resp_coordi.pose
                if True:
                    berick_object=[model,model_p.body_names[0]]
                    print berick_object
    #print resp_coordi
    '''
    controll_hz = rospy.Rate(5)
    while not rospy.is_shutdown():
        rospy.loginfo("all") 
        rospy.loginfo(box_name)
        controll_hz.sleep()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down")
    '''

if __name__ == '__main__':
    main(sys.argv)


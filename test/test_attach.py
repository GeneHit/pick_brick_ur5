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
from pick_brick_ur5.msg import Tracker
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse


from time import sleep

class brick_stack:
    def __init__(self):
        rospy.init_node("brick_stack", anonymous=False)

        self.pose_sub = rospy.Subscriber('brick_position', Floats,\
            self.tracking_callback, queue_size=1)
        #self.grasper_pub = rospy.Publisher('grasp_bool', Bool, queue_size=1)

        self.sysmode_pub = rospy.Publisher('sys_mode', String, queue_size=1)

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
        #******set the search strategic******
        #***set the search pose***
        self.search_pose = [ ]
        #search pose1
        self.search_pose.append(deepcopy(self.initial_pose))
        self.search_pose[0].position.x = 0.000
        self.search_pose[0].position.y = -0.200
        self.search_pose[0].position.z = 0.330
        self.search_pose[0].orientation.x = 1.0
        self.search_pose[0].orientation.y = 0.00
        self.search_pose[0].orientation.z = 0.00
        self.search_pose[0].orientation.w = 0.00
        #search pose2
        self.search_pose.append(deepcopy(self.initial_pose))
        self.search_pose[1].position.x = 0.000
        self.search_pose[1].position.y = -0.615
        self.search_pose[1].position.z = 0.330
        self.search_pose[1].orientation.x = 1.0
        self.search_pose[1].orientation.y = 0.00
        self.search_pose[1].orientation.z = 0.00
        self.search_pose[1].orientation.w = 0.00
        #search pose3
        self.search_pose.append(deepcopy(self.initial_pose))
        self.search_pose[2].position.x = 0.000
        self.search_pose[2].position.y = -0.821
        self.search_pose[2].position.z = 0.311
        self.search_pose[2].orientation.x = 1.0
        self.search_pose[2].orientation.y = 0.00
        self.search_pose[2].orientation.z = 0.00
        self.search_pose[2].orientation.w = 0.2303
        #set the search load
        #***set the search pose***endle
        #set the search load
        self.search_load = [0,1]
        #******set the search strategic******endle
        #set the brick pose
        self.object = deepcopy(self.initial_pose)
        self.object.orientation.x = 1.0
        self.object.orientation.y = 0.00
        self.object.orientation.z = 0.00
        self.object.orientation.w = 0.00
        
        #set the palce pose
        self.place_pose = deepcopy(self.initial_pose)
        self.place_pose.position.x = -0.7
        self.place_pose.position.y = 0.0
        self.place_pose.position.z = -0.035
        self.place_pose.orientation.x = 1.0
        self.place_pose.orientation.y = 0.00
        self.place_pose.orientation.z = 0.00
        self.place_pose.orientation.w = 0.00
        
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
        track_flag=int(msg.data[0])
        if track_flag == 1 :
            if np.isnan(msg.data[0]):
                pass
            else:
                self.object.position.x = msg.data[1]
                self.object.position.y = msg.data[2]
                self.object.position.z = msg.data[3]
                self.object.orientation.x = msg.data[4]
                self.object.orientation.y = msg.data[5]
                self.object.orientation.z = msg.data[6]
                self.object.orientation.w = msg.data[7]
                self.brick_flag=True
        else:
            self.brick_flag=False
        
def portal_plan(pose1,pose2):
    #  portal1 *----- * portal2
    #         -        -
    #        -          -
    # pose1 *------------* pose2
    t1=0.2
    t2=0.8
    portal1=deepcopy(pose1)
    portal2=deepcopy(pose2)
    portal1.position.x=pose1.position.x+t1*(pose2.position.x-pose1.position.x)
    portal1.position.y=pose1.position.y+t1*(pose2.position.y-pose1.position.y)
    portal1.position.z=pose1.position.z+t1*(pose2.position.z-pose1.position.z)+0.2
    portal2.position.x=pose1.position.x+t2*(pose2.position.x-pose1.position.x)
    portal2.position.y=pose1.position.y+t2*(pose2.position.y-pose1.position.y)
    portal2.position.z=pose1.position.z+t2*(pose2.position.z-pose1.position.z)+0.2
    portal_pose = [ ]
    portal_pose.append(deepcopy(portal1))
    portal_pose.append(deepcopy(portal2))
    portal_pose.append(deepcopy(pose2))
    return portal_pose


def main(args):
    controller=brick_stack()
    #if args[1]==True
    rospy.wait_for_service('/link_attacher_node/attach')
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',
                                    Attach)
    rospy.wait_for_service('/link_attacher_node/detach')
    detach_srv = rospy.ServiceProxy('/link_attacher_node/detach',
                                    Attach)
    rospy.wait_for_service('vacuum_pump_controll')
    pump_controll = rospy.ServiceProxy('vacuum_pump_controll',VacuumPump)
    pump_off = False
    while pump_off is False:
        try:
            pump_controll.wait_for_service()
            resp = pump_controll.call(False)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        if resp.isComplete is True:
            pump_off = True
    if True:
        controller.arm.set_joint_value_target(controller.intial_joint_states)
        # Set the internal state to the current state
        controller.arm.set_start_state_to_current_state()
        plan = controller.arm.plan()
        controller.arm.execute(plan)
    sysmode=int(0)
    search_flag= int(0)
    mode_1to0 = int(0)
    controller.sysmode_pub.publish(str(sysmode))
    controll_hz = rospy.Rate(15)
    while not rospy.is_shutdown():
        if sysmode == 0 :   #search the brick
            #rospy.loginfo("Go To the Initial Pose to Search Brick")
            rospy.loginfo("in 0")
            if search_flag == len(controller.search_load):
                search_flag =0
            if controller.brick_flag is True:
                sysmode=1
                controller.brick_flag=False
            else:
                controller.waypoints=[]
                controller.waypoints.append(deepcopy(controller.search_pose[controller.search_load[search_flag]]))
                # Set the internal state to the current state
                controller.arm.set_start_state_to_current_state()
                # Plan the Cartesian path(straint line) connecting the waypoints
                plan, fraction = controller.arm.compute_cartesian_path(controller.waypoints, 0.01, 0.0, True)
                # If we have a complete plan, execute the trajectory
                if fraction-0.2 < 0.7:
                    rospy.loginfo("Path planning failed")
                else:
                    rospy.loginfo("Path computed successfully. Moving the arm.")
                    num_pts = len(plan.joint_trajectory.points)
                    rospy.loginfo("\n# intermediate waypoints = "+str(num_pts))
                    controller.arm.execute(plan)
                    rospy.loginfo("Path execution complete.")
                    current_pose = controller.arm.get_current_pose(controller.end_effector_link).pose
                    if math.fabs(current_pose.position.x-controller.search_pose[controller.search_load[search_flag]].position.x)<0.05 and math.fabs(current_pose.position.y-controller.search_pose[controller.search_load[search_flag]].position.y)<0.05 and \
                        math.fabs(current_pose.position.z-controller.search_pose[controller.search_load[search_flag]].position.z)<0.05:
                        search_flag += 1
            if mode_1to0 != 0:
                sysmode = 1
                mode_1to0 += 1
            #publish sysymode=0,so that trigge the img process
            controller.sysmode_pub.publish(str(sysmode))
            sleep(0.02)
             
        elif sysmode == 1:  #plan to the brick pose
            rospy.loginfo("in 1")
            current_pose = controller.arm.get_current_pose(controller.end_effector_link).pose
            if math.fabs(current_pose.position.x-controller.object.position.x)<0.01 and \
                math.fabs(current_pose.position.y-controller.object.position.y)<0.01 and\
                math.fabs(current_pose.position.z-controller.object.position.z)<0.01:
                sysmode = 2
            else:
                controller.waypoints=[]
                controller.waypoints.append(deepcopy(controller.object))

                # Set the internal state to the current state
                controller.arm.set_start_state_to_current_state()

                # Plan the Cartesian path connecting the waypoints
                """moveit_commander.move_group.MoveGroupCommander.compute_cartesian_path(self, waypoints, eef_step, jump_threshold, avoid_collisios= True)

                   Compute a sequence of waypoints that make the end-effector move in straight line segments that follow the poses specified as waypoints.Configurations are computed for every eef_step meters;The jump_threshold specifies the maximum distance in configuration space between consecutive points in the resultingpath. The return value is a tuple: a fraction of how much of the path was followed,
                    the actual RobotTrajectory.
                """
                plan, fraction = controller.arm.compute_cartesian_path(controller.waypoints, 0.01, 0.0, True)
                # If we have a complete plan, execute the trajectory
                if fraction-0.2 > 0.7:
                    rospy.loginfo("Path computed successfully. Moving the arm.")
                    num_pts = len(plan.joint_trajectory.points)
                    rospy.loginfo("\n# intermediate waypoints = "+str(num_pts))
                    controller.arm.execute(plan)
                    rospy.loginfo("Path execution complete.")
                    mode_1to0 = 0
                else:
                    rospy.loginfo("Path planning failed\ncan't go to the brick")
                    #go to the mode 1 to change a pose to try to have a solution
                    sysmode = 0
                    if mode_1to0 == 0:
                        mode_1to0 = 1
                    if mode_1to0 == len(controller.search_load):
                        mode_1to0 = 0
        elif sysmode == 2:  #arrived the brick and pick the brick
            rospy.loginfo("in 2")
            #grasp_flag=True
            #controller.grasper_pub.publish(grasp_flag)
            try:
                #pump_controll.wait_for_service()
                resp = pump_controll.call(True)

                #attach_srv.wait_for_service()
                req = AttachRequest()
                req.model_name_1 = "robot"
                req.link_name_1 = "wrist_3_link"
                req.model_name_2 = "red_brick"
                req.link_name_2 = "brick_link"
                resp1 = attach_srv.call(req)
                print resp1

            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            if resp.isComplete is True:
                sysmode = 3
            else:
                pass
        elif sysmode == 3:  #plan to the place pose
            rospy.loginfo("in 3")
            current_pose = controller.arm.get_current_pose(controller.end_effector_link).pose
            if math.fabs(current_pose.position.x-controller.place_pose.position.x)<0.05 and  math.fabs(current_pose.position.y-controller.place_pose.position.y)<0.05 and math.fabs(current_pose.position.z-controller.place_pose.position.z)<0.05 :
                sysmode = 4
            else:
                waypoints=portal_plan(current_pose,controller.place_pose)

                #controller.waypoints=[]
                #controller.waypoints.append(deepcopy(waypoints[0]))
                #controller.waypoints.append(deepcopy(waypoints[1]))
                #controller.waypoints.append(deepcopy(waypoints[2]))
                #controller.waypoints=[]
                #controller.waypoints.append(deepcopy(controller.place_pose))
                # Set the internal state to the current state
                controller.arm.set_start_state_to_current_state()

                # Plan the Cartesian path(straint line) connecting the waypoints
                plan, fraction = controller.arm.compute_cartesian_path(waypoints, 0.01, 0.0, True)
                # If we have a complete plan, execute the trajectory
                if fraction-0.2 > 0.7:
                    num_pts = len(plan.joint_trajectory.points)
                    rospy.loginfo("\n# intermediate waypoints = "+str(num_pts))
                    #current_joint = controller.arm.get_current_joint_values()
                    #current_pose = controller.arm.get_current_pose()
                    plan1 = controller.arm.retime_trajectory(controller.robot.get_current_state(),plan,1.0)
                    rospy.loginfo("Path computed successfully. Moving the arm.")
                    num_pts = len(plan.joint_trajectory.points)
                    rospy.loginfo("\n# intermediate waypoints = "+str(num_pts))
                    controller.arm.execute(plan1)
                    rospy.loginfo("Path execution complete.")
                else:
                    rospy.loginfo("Path planning failed")
        elif sysmode == 4:  #arrived the place pose and place the brick
            rospy.loginfo("in 4")
            #grasp_flag=False
            #controller.grasper_pub.publish(grasp_flag)
            #sleep(0.005)
            #******close the vacuum pump******
            pump_off = False
            while pump_off is False:
                try:
                    #pump_controll.wait_for_service()
                    resp = pump_controll.call(False)

                    req = AttachRequest()
                    req.model_name_1 = "robot"
                    req.link_name_1 = "wrist_3_link"
                    req.model_name_2 = "red_brick"
                    req.link_name_2 = "brick_link"
                    detach_srv.call(req)
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e
                if resp.isComplete is True:
                    pump_off = True
            #end******close the vacuum pump******
            #******up 0.3m from place current pose
            current_pose = controller.arm.get_current_pose(controller.end_effector_link).pose
            #wpose = deepcopy(current_pose)
            current_pose.position.z = current_pose.position.z+0.3
            controller.waypoints=[]
            controller.waypoints.append(deepcopy(current_pose))
            controller.arm.set_start_state_to_current_state()
            plan, fraction = controller.arm.compute_cartesian_path(controller.waypoints, 0.01, 0.0, True)
            # If we have a complete plan, execute the trajectory
            if fraction-0.2 > 0.7:
                rospy.loginfo("Path computed successfully. Moving the arm.")
                num_pts = len(plan.joint_trajectory.points)
                rospy.loginfo("\n# intermediate waypoints = "+str(num_pts))
                controller.arm.execute(plan)
                rospy.loginfo("Path execution complete.")
            else:
                rospy.loginfo("Path planning failed")
            #******up 0.3m from place current pose*endle
            sysmode=0
        else:               #nothing to do
            pass

        controll_hz.sleep()


    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down")

if __name__ == '__main__':
    main(sys.argv)


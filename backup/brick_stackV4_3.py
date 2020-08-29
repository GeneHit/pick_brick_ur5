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
from gazebo_msgs.srv import GetWorldProperties,GetModelProperties,GetModelState

class brick_stack:
    def __init__(self):
        rospy.init_node("brick_stack", anonymous=False)

        self.pose_sub = rospy.Subscriber('brick_position', Floats,\
            self.tracking_callback, queue_size=1)
        #self.grasper_pub = rospy.Publisher('grasp_bool', Bool, queue_size=1)

        self.sysmode_pub = rospy.Publisher('sys_mode', String, queue_size=1)

        self.state_change_time = rospy.Time.now()

        rospy.loginfo("Starting brick_stack node moveit_cartesian_path")

        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        # Initialize the move group for the ur5_arm
        self.arm = moveit_commander.MoveGroupCommander('manipulator')

        #rospy.on_shutdown(self.cleanup)

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
        
        # Get the current pose so we can add it as a waypoint
        self.initial_pose = self.arm.get_current_pose(self.end_effector_link).pose
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
        self.search_pose[1].position.y = -0.4
        self.search_pose[1].position.z = 0.330
        self.search_pose[1].orientation.x = 1.0
        self.search_pose[1].orientation.y = 0.00
        self.search_pose[1].orientation.z = 0.00
        self.search_pose[1].orientation.w = 0.00
        #search pose3
        self.search_pose.append(deepcopy(self.initial_pose))
        self.search_pose[2].position.x = 0.000
        self.search_pose[2].position.y = -0.615
        self.search_pose[2].position.z = 0.330
        self.search_pose[2].orientation.x = 1.0
        self.search_pose[2].orientation.y = 0.00
        self.search_pose[2].orientation.z = 0.00
        self.search_pose[2].orientation.w = 0.00
        #search pose4
        self.search_pose.append(deepcopy(self.initial_pose))
        self.search_pose[3].position.x = 0.000
        self.search_pose[3].position.y = -0.8
        self.search_pose[3].position.z = 0.311
        self.search_pose[3].orientation.x = 1.0
        self.search_pose[3].orientation.y = 0.00
        self.search_pose[3].orientation.z = 0.00
        self.search_pose[3].orientation.w = 0.2303
        #set the search load
        #***set the search pose***endle
        #set the search load
        self.search_load = [0,1,2]
        #search again
        self.search_again = deepcopy(self.initial_pose)
        self.search_again.orientation.x = 1.0
        self.search_again.orientation.y = 0.00
        self.search_again.orientation.z = 0.00
        self.search_again.orientation.w = 0.00
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
        self.place_pose.position.z = -0.025
        self.place_pose.orientation.x = 1.0
        self.place_pose.orientation.y = 0.00
        self.place_pose.orientation.z = 0.00
        self.place_pose.orientation.w = 0.00
        
        self.brick_flag=False

        #use for open/close the vacuum pump in fuction: pump_contr(self,Open)
        rospy.wait_for_service('/link_attacher_node/attach')
        self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',Attach)
        rospy.wait_for_service('/link_attacher_node/detach')
        self.detach_srv = rospy.ServiceProxy('/link_attacher_node/detach',Attach)
        #rospy.wait_for_service('vacuum_pump_controll')
        #self.pump_controll = rospy.ServiceProxy('vacuum_pump_controll',VacuumPump)
        rospy.wait_for_service('/gazebo/get_world_properties')
        self.gazebo_world = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        rospy.wait_for_service('/gazebo/get_model_properties')
        self.model_prop = rospy.ServiceProxy('/gazebo/get_model_properties', GetModelProperties)
        rospy.wait_for_service('/gazebo/get_model_state')
        self.model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.brick_object=['red_brick_0','brick_link']

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
                self.search_again.position.x = msg.data[1]
                self.search_again.position.y = msg.data[2]+0.13
                self.search_again.position.z = msg.data[3]+0.2
                self.brick_flag=True
        else:
            self.brick_flag=False

    #****controll the vacuum pump function****
    def pump_contr(self,Open,object_pose,pose_flag):  
        ur5 = 'robot' #robot is the name of ur5 in gazebo
        ur5_end = "wrist_3_link"
        ok = False
        flag = False
        if pose_flag is True:
            #******get the object name to attach it******
            world = self.gazebo_world.call()
            red_brick = [ ]
            if world.success is True:
                for model in world.model_names:
                    if model.startswith('red_bric'):
                        red_brick.append(model)
                        #print red_brick
            #brick_object=[2]
            for model in red_brick:
                resp_s = self.model_state.call(model,ur5)
                #print resp_coordi
                if resp_s.success is True:
                    #print resp_coordi.pose
                    if math.sqrt( (resp_s.pose.position.x-object_pose.position.x)**2 + \
                        (resp_s.pose.position.y-object_pose.position.y)**2 + \
                        (resp_s.pose.position.z-object_pose.position.z)**2 ) < 0.04:
                        #0.03是半个砖块的距离
                        model_p = self.model_prop.call(model)
                        if model_p.success is True:
                            self.brick_object=[model,model_p.body_names[0]]
                            #print berick_object
                            flag = True
            if flag is False:
                print "there isn't a brick"
                return False
            #******get the object name to attach/detach it******endle
        req = AttachRequest()
        req.model_name_1 = ur5
        req.link_name_1 = ur5_end
        req.model_name_2 = self.brick_object[0]
        req.link_name_2 = self.brick_object[1]
        #******Grasp the object******
        if Open is True:
            while ok is False:
                try:
                    #pump_controll.wait_for_service()
                    #resp = self.pump_controll.call(True)
                    #attach_srv.wait_for_service()
                    resp1 = self.attach_srv.call(req)
                    ok = resp1.ok
                except rospy.ServiceException, e:
                    print "Pump Open Service call failed: %s"%e
            return True
        #******Grasp the object******endle
        #******Discard the object******
        if Open is False:
            while ok is False:
                try:
                    #pump_controll.wait_for_service()
                    #resp = self.pump_controll.call(False)
                    #attach_srv.wait_for_service()
                    resp1 = self.detach_srv.call(req)
                    ok = resp1.ok
                except rospy.ServiceException, e:
                    print "Pump Close Service call failed: %s"%e
        #******Discard the object******endle
        return True

    def go_cartesian_pose(self,wayposes,flag):
        if flag == 1:
            waypoints = [ ]
            waypoints.append(deepcopy(wayposes))
        else:
            waypoints = wayposes
        
        plan_flag=0
        while plan_flag <3:
            # Set the internal state to the current state
            self.arm.set_start_state_to_current_state()

            # Plan the Cartesian path connecting the waypoints
            """moveit_commander.move_group.MoveGroupCommander.compute_cartesian_path(self, waypoints, eef_step, jump_threshold, avoid_collisios= True)

               Compute a sequence of waypoints that make the end-effector move in straight line segments that follow the poses specified as waypoints.Configurations are computed for every eef_step meters;The jump_threshold specifies the maximum distance in configuration space between consecutive points in the resultingpath. The return value is a tuple: a fraction of how much of the path was followed,
                the actual RobotTrajectory.
            """
            plan, fraction = self.arm.compute_cartesian_path(waypoints, 0.01, 0.0, True)
            # If we have a complete plan, execute the trajectory
            if fraction-0.2 > 0.7:
                rospy.loginfo("Path computed successfully. Moving the arm.")
                num_pts = len(plan.joint_trajectory.points)
                rospy.loginfo("\n# intermediate waypoints = "+str(num_pts))
                self.arm.execute(plan)
                rospy.loginfo("Path execution complete.")
                return True
            else:
                rospy.loginfo("Path planning failed\ncan't go to the pose")
                plan_flag += 1
        return False

    def get_place_pose(self,place_num):
        # bricks placed by 2*2 matrix
        #   2-1
        #   4-3
        #brick size is 0.24*0.115*0.053 m
        brick_length=0.24+0.01
        brick_width=0.115+0.01
        brick_height=0.053
        center_x = -0.6
        center_y = 0.0
        center_z = -0.046 #0.053-0.1
        placex=[center_x-(brick_length/2.0),center_x+(brick_length/2.0)]
        placey=[center_y+(brick_width/2.0),center_y-(brick_width/2.0)]
        placez=[center_z,center_z+brick_height]
        numxy = (place_num-1)%4
        if numxy == 0:
            self.place_pose.position.x = placex[0]
            self.place_pose.position.y = placey[0]
        elif numxy == 1:
            self.place_pose.position.x = placex[0]
            self.place_pose.position.y = placey[1]
        elif numxy == 2:
            self.place_pose.position.x = placex[1]
            self.place_pose.position.y = placey[0]   
        else:
            self.place_pose.position.x = placex[1]
            self.place_pose.position.y = placey[1]

        numz = (place_num-1)/4
        self.place_pose.position.z = placez[0]+numz*(placez[1]-placez[0])
        
        
def portal_plan(pose1,pose2,t1,t2):
    #  portal1 *----- * portal2
    #         -        -
    #        -          -
    # pose1 *------------* pose2
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
    sys=brick_stack()
    place_num=int(3)
    #if args[1]==True
    '''
    pump_off = False
    try:
        sys.pump_controll.wait_for_service()
        resp = sys.pump_controll.call(False)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    '''
    if True:
        current_pose = sys.arm.get_current_pose(sys.end_effector_link).pose
        isComplete=sys.pump_contr(False,current_pose,True)
    if True:
        sys.arm.set_joint_value_target(sys.intial_joint_states)
        # Set the internal state to the current state
        sys.arm.set_start_state_to_current_state()
        plan = sys.arm.plan()
        sys.arm.execute(plan)
    sysmode=int(0)
    search_flag= int(0)
    mode_1to0 = int(0)
    mode_2to0 = int(0)
    sys.sysmode_pub.publish(str(sysmode))
    controll_hz = rospy.Rate(15)
    while not rospy.is_shutdown():
        #**0**********************search the brick*****************************
        if sysmode == 0 :   #search the brick
            rospy.loginfo("in 0")
            if search_flag == len(sys.search_load):
                search_flag =0
            if sys.brick_flag is True:
                sysmode=1
                sys.sysmode_pub.publish(str(sysmode))
                sys.brick_flag=False
            else:
                go_flag=sys.go_cartesian_pose( sys.search_pose[sys.search_load[search_flag]],1)
                search_flag += 1
            if mode_1to0 != 0:
                sysmode = 1
                mode_1to0 += 1
            if mode_2to0 !=0:
                sysmode = 2
                mode_2to0 += 1
            #publish sysymode=0,so that trigge the img process
            sys.sysmode_pub.publish(str(sysmode))
            sleep(0.02)
        #**0-endle*******************search the brick**************************
        #**1*****plan to the brick pose above 0.3m to search brick again*******    
        elif sysmode == 1:  #plan to the brick pose above 0.3m to search brick again
            rospy.loginfo("in 1")
            current_pose = sys.arm.get_current_pose(sys.end_effector_link).pose
            if math.sqrt((current_pose.position.x-sys.search_again.position.x)**2 +  \
                (current_pose.position.y-sys.search_again.position.y)**2 + \
                (current_pose.position.z-sys.search_again.position.z)**2 ) < 0.0001:
                print 'in 1-1'
                #sys.sysmode_pub.publish(str(0))
                if sys.brick_flag is True:
                    sysmode = 2
                    sys.sysmode_pub.publish(str(sysmode))
                    sys.brick_flag=False 
                else:
                    sysmode = 0              
            else:
                go_flag=sys.go_cartesian_pose(sys.search_again,1)
                if go_flag is True:
                    mode_1to0 = 0
                    sys.sysmode_pub.publish(str(0))
                    sleep(0.02)
                else:
                    #***go to the sysmode 0 to change a pose
                    #**try to have a solution to the brick
                    sysmode = 0
                    if mode_1to0 == 0:
                        mode_1to0 = 1
                    if mode_1to0 == len(sys.search_load):
                        mode_1to0 = 0
        #**1-endle***plan to the brick pose above 0.3m to search brick again*******
        #**2**************arrive the brick and pick the brick***************** 
        elif sysmode == 2:  #arrived the brick and pick the brick
            rospy.loginfo("in 2")
            current_pose = sys.arm.get_current_pose(sys.end_effector_link).pose
            if math.fabs(current_pose.position.x-sys.object.position.x)<0.001 and \
               math.fabs(current_pose.position.y-sys.object.position.y)<0.001 and \
               math.fabs(current_pose.position.z-sys.object.position.z)<0.001 :
                #open the vacuum pump
                isComplete=sys.pump_contr(True,current_pose,True)
                #isComplete=sys.pump_contr(True,sys.object,True)
                if isComplete is True:
                    sysmode = 3
                    place_num += 1
                else:
                    sysmode = 0
            else:
                go_flag=sys.go_cartesian_pose(sys.object,1)
                if go_flag is True:
                    pass
                else:
                    sysmode = 0
                    if mode_2to0 == 0:
                            mode_2to0 = 1
                    if mode_2to0 == len(sys.search_load):
                        mode_2to0 = 0
        #**2-endle********arrive the brick and pick the brick*****************
        #**3***********************plan to the place pose********************** 
        elif sysmode == 3:  #plan to the place pose
            rospy.loginfo("in 3")
            sys.get_place_pose(place_num)
            current_pose = sys.arm.get_current_pose(sys.end_effector_link).pose
            waypoints=portal_plan(current_pose,sys.place_pose,0.2,0.8)
            go_flag=sys.go_cartesian_pose(waypoints,2)
            if go_flag == True:
                sysmode = 4
        #**3-endle********************plan to the place pose**********************
        #**4***********arrived the place pose and place the brick**************
        elif sysmode == 4:  #arrived the place pose and place the brick
            rospy.loginfo("in 4")
            current_pose = sys.arm.get_current_pose(sys.end_effector_link).pose
            if math.fabs(current_pose.position.x-sys.place_pose.position.x)<0.001 and \
               math.fabs(current_pose.position.y-sys.place_pose.position.y)<0.001 and \
               math.fabs(current_pose.position.z-sys.place_pose.position.z)<0.001 :
                #close the vacuum pump
                current_pose = sys.arm.get_current_pose(sys.end_effector_link).pose
                isComplete=sys.pump_contr(False,current_pose,False)
                #isComplete=sys.pump_contr(False)
                if isComplete == True:
                    sysmode = 5
            else:
                pass
        #**4-endle*********arrived the place pose and place the brick**************
        #**5****************up 0.3m  and go to the innitial pose*******************
        elif sysmode == 5:
            rospy.loginfo("in 5")
            current_pose = sys.arm.get_current_pose(sys.end_effector_link).pose
            if \
            math.fabs(current_pose.position.x-sys.search_pose[0].position.x)<0.001 and \
            math.fabs(current_pose.position.y-sys.search_pose[0].position.y)<0.001 and \
            math.fabs(current_pose.position.z-sys.search_pose[0].position.z)<0.001 :
                sysmode=0
            else:
                current_pose.position.z = current_pose.position.z+0.1
                waypoints=[ ]
                waypoints.append(deepcopy(current_pose))
                waypoints.append(deepcopy(sys.search_pose[0]))
                go_flag=sys.go_cartesian_pose(waypoints,2)
                if go_flag is True:
                    pass
                else:
                    pass

        #**5-endle***********up 0.3m  and go to the innitial pose******************
        #**6******nothing to do********
        else:
            pass
        #**6-endle*********************

        controll_hz.sleep()


    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down")

if __name__ == '__main__':
    main(sys.argv)


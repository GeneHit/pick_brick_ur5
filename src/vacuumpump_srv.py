#!/usr/bin/env python

#import roslib
#roslib.load_manifest("pick_brick_ur5")
import rospy, sys, numpy as np
import geometry_msgs.msg
import moveit_msgs.msg
from std_msgs.msg import Header
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from pick_brick_ur5.srv import VacuumPump

def pump_on(num_th):
    #service /ur5/vacuum_gripper0/on
    srv_str='/ur5/vacuum_gripper'+str(num_th)+'/on'
    # Wait till the srv is available
    rospy.wait_for_service(srv_str)
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy(srv_str, Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def pump_off(num_th):
    srv_str='/ur5/vacuum_gripper'+str(num_th)+'/off'
    # Wait till the srv is available
    rospy.wait_for_service(srv_str)
    try:
        # Create a handle for the calling the srv
        turn_off = rospy.ServiceProxy(srv_str, Empty)
        # Use this handle just like a normal function and call it
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def trigger_on():
    for i in range(9):
        pump_on(int(i))

def trigger_off():
    for i in range(9):
        pump_off(int(i))

def pump_callback(req):
    if req.PumpOn is True:
        trigger_on()
        return True
    else:
        trigger_off()
        return True

def vacuum_pump_srv(args):
    rospy.init_node("vacuum_pump", anonymous=False)
    rospy.loginfo("\n***********vacuum_pump controll node***********")
    pump_srv = rospy.Service('vacuum_pump_controll', VacuumPump, pump_callback)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down")

if __name__ == '__main__':
    vacuum_pump_srv(sys.argv)    


#!/usr/bin/env python



import rospy, sys, numpy as np
import geometry_msgs.msg
import moveit_msgs.msg
from std_msgs.msg import Header
from std_msgs.msg import Bool
from std_srvs.srv import Empty

def gripper0_off():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper0/off')
    try:
        # Create a handle for the calling the srv
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper0/off', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper1_off():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper1/off')
    try:
        # Create a handle for the calling the srv
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper1/off', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper2_off():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper2/off')
    try:
        # Create a handle for the calling the srv
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper2/off', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper3_off():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper3/off')
    try:
        # Create a handle for the calling the srv
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper3/off', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper4_off():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper4/off')
    try:
        # Create a handle for the calling the srv
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper4/off', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper5_off():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper5/off')
    try:
        # Create a handle for the calling the srv
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper5/off', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper6_off():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper6/off')
    try:
        # Create a handle for the calling the srv
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper6/off', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper7_off():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper7/off')
    try:
        # Create a handle for the calling the srv
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper7/off', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper8_off():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper8/off')
    try:
        # Create a handle for the calling the srv
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper8/off', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper9_off():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper9/off')
    try:
        # Create a handle for the calling the srv
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper9/off', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper10_off():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper10/off')
    try:
        # Create a handle for the calling the srv
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper10/off', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper11_off():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper11/off')
    try:
        # Create a handle for the calling the srv
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper11/off', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper12_off():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper12/off')
    try:
        # Create a handle for the calling the srv
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper12/off', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper13_off():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper13/off')
    try:
        # Create a handle for the calling the srv
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper13/off', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper14_off():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper14/off')
    try:
        # Create a handle for the calling the srv
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper14/off', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper15_off():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper15/off')
    try:
        # Create a handle for the calling the srv
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper15/off', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper16_off():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper16/off')
    try:
        # Create a handle for the calling the srv
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper16/off', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper17_off():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper17/off')
    try:
        # Create a handle for the calling the srv
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper17/off', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper18_off():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper18/off')
    try:
        # Create a handle for the calling the srv
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper18/off', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper19_off():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper19/off')
    try:
        # Create a handle for the calling the srv
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper19/off', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper20_off():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper20/off')
    try:
        # Create a handle for the calling the srv
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper20/off', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper21_off():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper21/off')
    try:
        # Create a handle for the calling the srv
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper21/off', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper22_off():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper22/off')
    try:
        # Create a handle for the calling the srv
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper22/off', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper23_off():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper23/off')
    try:
        # Create a handle for the calling the srv
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper23/off', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper24_off():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper24/off')
    try:
        # Create a handle for the calling the srv
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper24/off', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def trigger_off():
    gripper0_off()
    gripper1_off()
    gripper2_off()
    gripper3_off()
    gripper4_off()
    gripper5_off()
    gripper6_off()
    gripper7_off()
    gripper8_off()
    '''
    gripper9_off()
    gripper10_off()
    gripper11_off()
    gripper12_off()
    gripper13_off()
    gripper14_off()
    gripper15_off()
    gripper16_off()
    gripper17_off()
    gripper18_off()
    gripper19_off()
    gripper20_off()
    gripper21_off()
    gripper22_off()
    gripper23_off()
    gripper24_off() 
    '''

if __name__ == '__main__':
    rospy.init_node("ur5_gripper_off", anonymous=False)
    trigger_off()

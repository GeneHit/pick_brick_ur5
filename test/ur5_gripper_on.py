#!/usr/bin/env python



import rospy, sys, numpy as np
import geometry_msgs.msg
import moveit_msgs.msg
from std_msgs.msg import Header
from std_msgs.msg import Bool
from std_srvs.srv import Empty

def gripper0_on():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper0/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur5/vacuum_gripper0/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper1_on():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper1/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur5/vacuum_gripper1/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper2_on():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper2/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur5/vacuum_gripper2/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper3_on():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper3/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur5/vacuum_gripper3/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper4_on():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper4/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur5/vacuum_gripper4/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper5_on():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper5/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur5/vacuum_gripper5/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper6_on():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper6/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur5/vacuum_gripper6/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper7_on():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper7/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur5/vacuum_gripper7/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper8_on():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper8/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur5/vacuum_gripper8/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper9_on():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper9/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur5/vacuum_gripper9/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper10_on():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper10/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur5/vacuum_gripper10/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper11_on():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper11/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur5/vacuum_gripper11/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper12_on():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper12/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur5/vacuum_gripper12/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper13_on():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper13/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur5/vacuum_gripper13/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper14_on():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper14/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur5/vacuum_gripper14/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper15_on():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper15/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur5/vacuum_gripper15/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper16_on():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper16/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur5/vacuum_gripper16/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper17_on():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper17/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur5/vacuum_gripper17/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper18_on():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper18/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur5/vacuum_gripper18/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper19_on():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper19/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur5/vacuum_gripper19/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper20_on():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper20/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur5/vacuum_gripper20/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper21_on():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper21/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur5/vacuum_gripper21/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper22_on():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper22/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur5/vacuum_gripper22/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper23_on():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper23/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur5/vacuum_gripper23/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper24_on():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper24/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur5/vacuum_gripper24/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper0_off():
    rospy.wait_for_service('/ur5/vacuum_gripper0/off')
    try:
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper0/off', Empty)
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper1_off():
    rospy.wait_for_service('/ur5/vacuum_gripper1/off')
    try:
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper1/off', Empty)
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper2_off():
    rospy.wait_for_service('/ur5/vacuum_gripper2/off')
    try:
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper2/off', Empty)
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper3_off():
    rospy.wait_for_service('/ur5/vacuum_gripper3/off')
    try:
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper3/off', Empty)
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper4_off():
    rospy.wait_for_service('/ur5/vacuum_gripper4/off')
    try:
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper4/off', Empty)
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper5_off():
    rospy.wait_for_service('/ur5/vacuum_gripper5/off')
    try:
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper5/off', Empty)
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper6_off():
    rospy.wait_for_service('/ur5/vacuum_gripper6/off')
    try:
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper6/off', Empty)
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper7_off():
    rospy.wait_for_service('/ur5/vacuum_gripper7/off')
    try:
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper7/off', Empty)
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper8_off():
    rospy.wait_for_service('/ur5/vacuum_gripper8/off')
    try:
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper8/off', Empty)
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def trigger_on():
    gripper0_on()
    gripper1_on()
    gripper2_on()
    gripper3_on()
    gripper4_on()
    gripper5_on()
    gripper6_on()
    gripper7_on()
    gripper8_on()
    '''
    gripper9_on()
    gripper10_on()
    gripper11_on()
    gripper12_on()
    gripper13_on()
    gripper14_on()
    gripper15_on()
    gripper16_on()
    gripper17_on()
    gripper18_on()
    gripper19_on()
    gripper20_on()
    gripper21_on()
    gripper22_on()
    gripper23_on()
    gripper24_on()
    '''
       
if __name__ == '__main__':
    rospy.init_node("ur5_gripper_on", anonymous=False)
    trigger_on()


#!/usr/bin/env python  
import roslib
#roslib.load_manifest('pick_brick_ur5')
import rospy
import math
import geometry_msgs.msg
#from rospy_tutorials.msg import Float
from std_msgs.msg import String
#import turtlesim.srv

def tracking_callback(msg):
    mode=int(msg.data) 
    if mode == 2:   
        print(mode)
    else:
        pass

if __name__ == '__main__':
    rospy.init_node('sys_mode_listener')
    sysmode_pub = rospy.Subscriber('sys_mode', String, tracking_callback, queue_size=1)

    while not rospy.is_shutdown():
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            rospy.loginfo("Shutting down")


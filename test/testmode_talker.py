#!/usr/bin/env python  
import roslib
#roslib.load_manifest('pick_brick_ur5')
import rospy
import math
import geometry_msgs.msg
#from rospy_tutorials.msg import Float
from std_msgs.msg import String
#import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('sys_mode_client')
    sysmode_pub = rospy.Publisher('sys_mode', String, queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            mode=2
            sysmode_pub.publish(str(mode))
        except rospy.ROSInterruptException:
            continue
        rate.sleep()


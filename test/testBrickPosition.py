#!/usr/bin/env python

"""
    moveit_cartesian_path.py - Version 0.1 2016-07-28

    Based on the R. Patrick Goebel's moveit_cartesian_demo.py demo code.

    Plan and execute a Cartesian path for the end-effector.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html
"""

import rospy, sys, numpy as np
import moveit_commander
from copy import deepcopy
import geometry_msgs.msg
from std_msgs.msg import Header
from rospy_tutorials.msg import Floats

class go_pick_pose:
    def __init__(self):
        rospy.init_node("go_pick_pose", anonymous=True)

        self.cxy_sub = rospy.Subscriber('brick_position', Floats, self.tracking_callback, queue_size=1)


    def tracking_callback(self,msg):
        rospy.loginfo(msg)
        #flag=msg.data[0]
        #rospy.loginfo("flag:")
        #rospy.loginfo(flag)

def main(args):
    mp=go_pick_pose()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

if __name__ == '__main__':
    main(sys.argv)

#!/usr/bin/env python  
import roslib
roslib.load_manifest('pick_brick_ur5')
import rospy
import math
import tf
import geometry_msgs.msg
#import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('kinect_tf_listener')

    listener = tf.TransformListener()

    '''
    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    spawner(4, 2, 0, 'turtle2')
    #turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)
    '''

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/base_link', '/kinect_link', rospy.Time(0))
            print "----trans----\n",trans,"\n----rot----\n",rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()

    '''
    angular = 4 * math.atan2(trans[1], trans[0])
    linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
    cmd = geometry_msgs.msg.Twist()
    cmd.linear.x = linear
    cmd.angular.z = angular
    turtle_vel.publish(cmd)
    '''


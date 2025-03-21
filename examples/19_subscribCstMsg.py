#!/usr/bin/env python

import rospy
from pack.msg import Complex

def callback(msg):
    print("Real: {}".format(msg.real))
    print("Imaginary: {}".format(msg.imaginary))

rospy.init_node("message subscriber")
sub = rospy.Subscriber("complex", Complex, callback)
rospy.spin()

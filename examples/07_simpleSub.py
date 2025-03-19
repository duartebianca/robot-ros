#! /usr/bin/python3

import rospy
from std_msgs.msg import Int32


# Define the function called by the subscriber
def callback(msg):
    print (msg.data)

rospy.init_node('simple_subscriber')

# Define the new subsriber
rospy.Subscriber('counter', Int32, callback)

# Equivalent to an infinite while to not close the program
rospy.spin()

#! /usr/bin/python3

import rospy
import time
from std_msgs.msg import Int32

# Define the function called by the subscriber
def callback(msg):
    print msg.data
    time.sleep(1) # Simulate some processing

rospy.init_node('simple_subscriber')

# queue_size is used to read the last message
rospy.Subscriber('counter', Int32, callback, queue_size=1)

rospy.spin()

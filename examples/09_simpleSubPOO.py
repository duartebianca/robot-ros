#! /usr/bin/python3

import rospy
from std_msgs.msg import Int32

# Definition of the class
class mySub():

    def __init__(self):
        # Define the subscriber
        self.sub = rospy.Subscriber('counter', Int32, self.callback, queue_size=1)

        # Message variables
        self.counterValue = 0

    # Definition of the function called by the subscriber
    def callback(self, msg):
        self.counterValue = msg.data

    def printMsg(self):
        print(self.counterValue)

# Main program
if __name__ == '__main__':
    # Define the node
    rospy.init_node('simpleSubPOO')
    # Create an object of class mySub and run the init function
    subObj = mySub()
    # While ROS is running
    while not rospy.is_shutdown():
        # Call printMsg method of mySub class
        subObj.printMsg()

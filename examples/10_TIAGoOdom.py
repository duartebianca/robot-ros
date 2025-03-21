#! /usr/bin/python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Definition of the class
class mySub():

    def __init__(self):
        # Define the subscriber
        self.sub = rospy.Subscriber('/mobile_base_controller/odom', Odometry, self.callback, queue_size=1)

    # Definition of the function called by the subscriber
    def callback(self, msg):
        qtn = msg.pose.pose.orientation
        qtn_list = [qtn.x, qtn.y, qtn.z, qtn.w]
        (roll, pitch, yaw) = euler_from_quaternion (qtn_list)
        print(roll, pitch, yaw)

# Main program
if __name__ == '__main__':
    # Define the node
    rospy.init_node('TIAGo_odem_node')
    # Create an object of class mySub and run the init function
    subObj = mySub()
    # While ROS is running
    rospy.spin()


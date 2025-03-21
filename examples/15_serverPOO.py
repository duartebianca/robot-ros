#! /usr/bin/python

import rospy
import math

from aula_offline.srv import turtleDistance,turtleDistanceResponse
from turtlesim.msg import Pose

class myServerClass():

    x, y = 0, 0

    def __init__(self):
        self.sub = rospy.Subscriber('/turtle1/pose', Pose, self.turtlePose_callback, queue_size=1)
        self.my_service = rospy.Service('turtleDistance_service', turtleDistance, self.handle_turtleDistance)
        print("Ready for turtle distance")

    def turtlePose_callback(self, msg):
        self.x = msg.x
        self.y = msg.y

    def handle_turtleDistance(self,req):
        turtleDistanceResponse = math.sqrt(self.x*self.x + self.y*self.y)
        return turtleDistanceResponse

#-------------------------
if __name__ == "__main__":

    rospy.init_node("server_node")
    myServer = myServerClass()
    rospy.spin()

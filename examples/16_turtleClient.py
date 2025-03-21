#! /usr/bin/python

import rospy

from aula_offline.srv import *


if __name__ == "__main__":

    rospy.wait_for_service('turtleDistance_service')
    print("turtleDistance_service available")

    try:
        h_turleDistance = rospy.ServiceProxy('turtleDistance_service', turtleDistance)
        response = h_turleDistance()
        print(response.Distance)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

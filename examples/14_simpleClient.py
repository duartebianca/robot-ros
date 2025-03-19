#! /usr/bin/python3

import rospy
from aula_offline.srv import *

if __name__ == "__main__":

    # Wait for the server to be started
    rospy.wait_for_service('add_two_ints_service_name')
    print("add_two_ints service is active")

    try:
        # Connect to the server
        h_AddTwoInts = rospy.ServiceProxy('add_two_ints_service_name', AddTwoInts)

        # Create and fill the resquest
        request = AddTwoIntsRequest()
        print('Service call')
        request.A = 2
        request.B = 3
        # Call the service
        response = h_AddTwoInts(request)
        print(response.Sum)

    # If the connection fails (DEBUG)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

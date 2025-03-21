#! /usr/bin/python3

import rospy

# Import the service classes: servive definition, response message
from pkg_name.srv import AddTwoInts, AddTwoIntsResponse

# Definition of the function called by the serice
def callback_AddTwoInts(req):

    # Create a response variable
    res = AddTwoIntsResponse()
    # Compute the sum
    res.sum = req.A + req.B
    # Return the response variable
    return res

if __name__ == "__main__":

    # Start the node
    rospy.init_node("AddTwoInts_server_node")
    # Start the service server
    my_service = rospy.Service('add_two_ints_service_name', AddTwoInts, callback_AddTwoInts)

    print("Ready to add two ints.")

    # Wait to be closed by the user
    rospy.spin()

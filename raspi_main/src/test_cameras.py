#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerRequest

class TestCameras:

    def __init__(self):
        """
        Class constructor
        """
        # self.disc_name = "disc123"

        # Initialize node, name it 'test_cameras'
        rospy.init_node('test_cameras', anonymous=True)

        # Wait for the services that we use to be ready
        rospy.wait_for_service('/cameras_takephoto')
        rospy.loginfo("test_cameras node ready")

    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """

        # Use /cameras_takephoto service
        request = TriggerRequest()
        cameras_service = rospy.ServiceProxy('/cameras_takephoto', Trigger)

        # Call the service and get the response
        response = cameras_service(request)
        rospy.loginfo("Returned from cameras_service call, response is : %s\n", response.message)

        # Call the service and get the response
        response = cameras_service(request)
        rospy.loginfo("Returned from cameras_service call, response is : %s\n", response.message)

        rospy.spin()

if __name__ == '__main__':    
    TestCameras().run()
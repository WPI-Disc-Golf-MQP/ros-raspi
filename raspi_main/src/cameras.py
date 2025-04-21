#!/usr/bin/env python3

## This ROS node captures images from the cameras

import rospy
import cv2
import datetime
import os

from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

DEBUG: bool = True

class Cameras:

    def __init__(self):
        """
        Class constructor
        """
        self.disc_name = "disc123"
        self.camera1_device = 0
        self.camera2_device = 2
        self.camera3_device = 4
        self.photo_dir = "/tmp/photos/"

        # Initialize the node and call it "cameras"
        rospy.init_node("cameras", anonymous=True)

        # Create a new service called "cameras_takephoto" that accepts messages of
        # type Trigger and calls self.take_photos() when a message is received
        rospy.Service('cameras_takephoto', Trigger, self.take_photos)

        # Create the photos directory if it doesn't exist
        os.makedirs(self.photo_dir, exist_ok=True)

        rospy.loginfo("cameras node ready")

    def take_photos(self, msg: TriggerRequest) -> TriggerResponse:
        """
        Takes photos of the disc.

        Multiple cameras don't work unless you release a camera before opening the next camera. The
        documentation says that you should be able to open multiple cameras, use camera.grab() on all
        of them to grab a frame and then use camera.retrieve() to get the images. This always ended
        with blank images unless the camera was released before a new camera was used.

        If there are problems with the images, add a sleep before doing the camera.read() to give the
        camera enough time to set up.
        """ 
        rospy.loginfo("Cameras.take_photos() called")

        # Get current date and time
        now = datetime.datetime.now()

        # Use date & time to create filenames
        filename = self.photo_dir + self.disc_name + "-" + now.strftime("%Y%m%d-%H%M%S")
        if DEBUG:
            rospy.loginfo("filename is %s", filename)

        camera1 = cv2.VideoCapture(self.camera1_device)
        # rospy.sleep(1)
        success, image1 = camera1.read()
        if DEBUG:
            rospy.loginfo("camera1.read retval is %s", success)
        camera1.release()

        camera2 = cv2.VideoCapture(self.camera2_device)
        # rospy.sleep(1)
        success, image2 = camera2.read()
        if DEBUG:
            rospy.loginfo("camera2.read retval is %s", success)
        camera2.release()

        camera3 = cv2.VideoCapture(self.camera3_device)
        # rospy.sleep(1)
        success, image3 = camera3.read()
        if DEBUG:
            rospy.loginfo("camera3.read retval is %s", success)
        camera3.release()

        # resize and display images
        if DEBUG:
            image1 = cv2.resize(image1, (0,0), fx=0.5, fy=0.5)
            image2 = cv2.resize(image2, (0,0), fx=0.5, fy=0.5)
            image3 = cv2.resize(image3, (0,0), fx=0.5, fy=0.5)
            cv2.imshow("Camera 1", image1)
            cv2.imshow("Camera 2", image2)
            cv2.imshow("Camera 3", image3)
            cv2.waitKey(5000)
            cv2.destroyAllWindows()
        
        # save images
        cv2.imwrite(filename+"-1.jpg", image1)
        cv2.imwrite(filename+"-2.jpg", image2)
        cv2.imwrite(filename+"-3.jpg", image3)

        response = TriggerResponse()
        response.success = True
        response.message = "Done"

        return response
    
    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.spin()

if __name__ == '__main__':
    Cameras().run()
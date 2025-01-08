#!/usr/bin/env python3
from __future__ import annotations

from threading import Timer
from typing import Callable

import rospy
from std_msgs.msg import Int8, Float32, Bool

from node_templates import *

import cv2
import os
import numpy

class PHOTOBOOTH_STATE(Enum):
    PHOTOBOOTH_IDLE = 0
    PHOTOBOOTH_RISING = 1
    PHOTOBOOTH_TURNING = 2
    PHOTOBOOTH_LOWERING = 3

DEBUG: bool = False

# hal__photobooth inherits from measure_node, which is imported from node_templates, but has not been implemented
class hal__photobooth(measure_node):

    def __init__(self, completion_callback:Callable[[str], None]=lambda _: None):
        """
        Class constructor
        Can be called with an optional callback function that takes a string and returns nothing
        """
        
    
        self.camera1_device = 0
        self.camera2_device = 2
        self.camera3_device = 2

        # This calls the parent class's (measure_node) constructor, but measure_node has not been implemented
        super().__init__(NAME="photobooth", STATE_TYPE=PHOTOBOOTH_STATE, 
                         COMPLETION_CALLBACK=completion_callback)


    def state_update(self, msg:Int8):
        """
        Updates self.state
        :param msg    [Int8]    The state to assign using PHOTOBOOTH_STATE
        """
        self.state = PHOTOBOOTH_STATE(msg.data)
    
    def get_state(self) -> int:
        """
        Gets current state
        :return     [int]   The current state (to interpret with PHOTOBOOTH_STATE)
        """
        return self.state.value
    
    def picture_disc(self,camera):
        rospy.loginfo("picture_disc is OBSOLETE, use get_images instead")
    
    
    def get_images(self) -> tuple[numpy.array, numpy.array, numpy.array]:
        """
        Grabs images from the cameras

        Multiple cameras don't work unless you release a camera before opening the next camera. The
        documentation says that you should be able to open multiple cameras, use camera.grab() on all
        of them to grab a frame and then use camera.retrieve() to get the images. This always ended
        with blank images unless the camera was released before a new camera was used.

        If there are problems with the images, add a sleep before doing the camera.read() to give the
        camera enough time to set up.

        :return     tuple[numpy.array, numpy.array, numpy.array]    The images from the cameras.
        """
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

        return [image1, image2, image3]


    def camera_info(self) -> None:
        """
        Logs the camera info
        """
        rospy.loginfo("Camera 1 is /dev/video%s", self.camera1_device)
        camera1 = cv2.VideoCapture(self.camera1_device)
        rospy.loginfo("Camera 1 Width: %s", camera1.get(cv2.CAP_PROP_FRAME_WIDTH))
        rospy.loginfo("Camera 1 Height: %s", camera1.get(cv2.CAP_PROP_FRAME_HEIGHT))
        rospy.loginfo("Camera 1 FPS: %s", camera1.get(cv2.CAP_PROP_FPS))
        camera1.release()

        rospy.loginfo("Camera 2 is /dev/video%s", self.camera2_device)
        camera2 = cv2.VideoCapture(self.camera2_device)
        rospy.loginfo("Camera 2 Width: %s", camera2.get(cv2.CAP_PROP_FRAME_WIDTH))
        rospy.loginfo("Camera 2 Height: %s", camera2.get(cv2.CAP_PROP_FRAME_HEIGHT))
        rospy.loginfo("Camera 2 FPS: %s", camera2.get(cv2.CAP_PROP_FPS))
        camera2.release()

        rospy.loginfo("Camera 3 is /dev/video%s", self.camera3_device)
        camera3 = cv2.VideoCapture(self.camera3_device)
        rospy.loginfo("Camera 3 Width: %s", camera3.get(cv2.CAP_PROP_FRAME_WIDTH))
        rospy.loginfo("Camera 3 Height: %s", camera3.get(cv2.CAP_PROP_FRAME_HEIGHT))
        rospy.loginfo("Camera 3 FPS: %s", camera3.get(cv2.CAP_PROP_FPS))
        camera3.release()


if __name__ == '__main__':
    rospy.init_node('hal__photobooth')
    rospy.loginfo("hal__photobooth node started")

    DEBUG = True

    # clean up on shutdown
    #rospy.on_shutdown(hal__photobooth.shutdown_hook)

    def _completion_callback(_):
        print("* Movement Complete, State: " + str(photobooth.get_state()) + ", notified via callback.")

    photobooth = hal__photobooth(_completion_callback)

    rospy.loginfo("photobooth node is running, start tests")

    # wait for SAMD to start
    rospy.sleep(5)

    rospy.loginfo("start state machine")   
    photobooth.start()

    while not rospy.is_shutdown():
        rospy.sleep(1)
        # do stuff

    #     # Get camera info
    #     rospy.loginfo("###################")
    #     rospy.loginfo("# Get camera info #")
    #     rospy.loginfo("###################")
    #     photobooth.camera_info()

    #     # Check cameras
    #     rospy.loginfo("####################")
    #     rospy.loginfo("# Checking cameras #")
    #     rospy.loginfo("####################")
    #     [image1, image2, image3] = photobooth.get_images()

    #     # resize and display images
    #     image1 = cv2.resize(image1, (0,0), fx=0.5, fy=0.5)
    #     image2 = cv2.resize(image2, (0,0), fx=0.5, fy=0.5)
    #     image3 = cv2.resize(image3, (0,0), fx=0.5, fy=0.5)
    #     cv2.imshow("Camera 1", image1)
    #     cv2.imshow("Camera 2", image2)
    #     cv2.imshow("Camera 3", image3)
    #     cv2.waitKey(5000)
    #     cv2.destroyAllWindows()

    #     # if complete, wait until shutdown
    #     if (photobooth.complete()):
    #         rospy.sleep(1)

    # rospy.loginfo("Shutting down...")
    # cv2.destroyAllWindows()
    



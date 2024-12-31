#!/usr/bin/env python3

from threading import Timer
from typing import Callable

import rospy
from std_msgs.msg import Int8, Float32, Bool

from node_templates import *

import cv2
import os
import numpy

# cam10degrees = cv2.VideoCapture(1)
# cam35degrees = cv2.VideoCapture(4)

class TURNTABLE_STATE(Enum):
    TURNTABLE_IDLE = 0
    RAISING = 1
    SPINNING_PHOTOS = 2
    LOWERING = 3

DEBUG: bool = False

# hal__turntable inherits from measure_node, which is imported from node_templates, but has not been implemented
class hal__turntable(measure_node):

    def __init__(self, completion_callback:Callable[[str], None]=lambda _: None):
        """
        Class constructor
        Can be called with an optional callback function that takes a string and returns nothing
        """
        rospy.loginfo("hal__turntable node started")
    
        self.camera1_device = 0
        self.camera2_device = 2
        self.camera3_device = 2

        # This calls the parent class's (measure_node) constructor, but measure_node has not been implemented
        super().__init__(NAME="turntable", STATE_TYPE=TURNTABLE_STATE, 
                         COMPLETION_CALLBACK=completion_callback)


    def state_update(self, msg:Int8):
        """
        Updates self.state
        :param msg    [Int8]    The state to assign using TURNTABLE_STATE
        """
        self.state = TURNTABLE_STATE(msg.data)
    
    def get_state(self) -> int:
        """
        Gets current state
        :return     [int]   The current state (to interpret with TURNTABLE_STATE)
        """
        return self.state.value
    
    # def picture_disc(self,camera):
    #     """
    #     ???
    #     :param camera    [VideoCapture]    A camera being accessed via OpenCV
    #     """
    #     # os.mkdir(dirname)
    #     # for i in range(0, 10):
    #     #     # cam10degrees.set(cv2.CAP_PROP_FRAME_WIDTH, 1200)
    #     #     # cam10degrees.set(cv2.CAP_PROP_FRAME_HEIGHT, 1920)
    #     #     # cam10degrees.set(cv2.CAP_PROP_FPS, 90)

    #     #     cam35degrees.set(cv2.CAP_PROP_FRAME_WIDTH, 1200)
    #     #     cam35degrees.set(cv2.CAP_PROP_FRAME_HEIGHT, 1920)
    #     #     cam35degrees.set(cv2.CAP_PROP_FPS, 90)

    #     #     # ret, image = cam10degrees.read()
    #     #     success, image2 = cam35degrees.read()

    #     #     # name10Degrees = str(10) + ' ' + discName + str(i+1) + '.jpg'
    #     #     name35Degrees = str(35) + ' ' + discName + str(i+1) + '.jpg'

    #     #     # cv2.imwrite(os.path.join(dirname, name10Degrees), img=image)
    #     #     cv2.imwrite(os.path.join(dirname, name35Degrees), img=image2)

    #     #     rospy.loginfo(os.path.join(dirname, name35Degrees))
    #     #     cv2.waitKey(200)

    #     # # cam10degrees.release()
    #     # cam35degrees.release()

    #     # Grabs, decodes and returns the next video frame.
    #     success, image = camera.read()

    #     # Set camera properties (why do this after grabbing a frame?)
    #     # camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1200)
    #     # camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 1920)
    #     # camera.set(cv2.CAP_PROP_FPS, 90)
    #     camera.set(cv2.CAP_PROP_FRAME_WIDTH, 600)
    #     camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 860)
    #     camera.set(cv2.CAP_PROP_FPS, 1)

    #     # Show the image in a window. Closes when the user presses a key or 5 seconds pass
    #     cv2.imshow("image", image)
    #     cv2.waitKey(5000)

    #     # cv2.imwrite("testPhoto.jpg", image)
    #     # rospy.loginfo("saving image?")

    #     # Release the video camera and free system resources
    #     camera.release()

    #     # rospy.loginfo("should be saving images")

    #     # Make sure that all OpenCV windows get closed
    #     cv2.destroyAllWindows()
    
    def get_images(self) -> None:
        """
        Grabs images from the cameras
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

        image1 = cv2.resize(image1, (0,0), fx=0.5, fy=0.5)
        image2 = cv2.resize(image2, (0,0), fx=0.5, fy=0.5)
        image3 = cv2.resize(image3, (0,0), fx=0.5, fy=0.5)

        if DEBUG:
            cv2.imshow("Camera 1", image1)
            cv2.imshow("Camera 2", image2)
            cv2.imshow("Camera 3", image3)
            cv2.waitKey(5000)
            cv2.destroyAllWindows()


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
    rospy.init_node('hal__turntable')
    rospy.loginfo("hal__turntable node started")

    DEBUG = True

    # clean up on shutdown
    #rospy.on_shutdown(hal__turntable.shutdown_hook)

    def _completion_callback(_):
        print("* Movement Complete, State: " + str(turntable.get_state()) + ", notified via callback.")

    turntable = hal__turntable(_completion_callback)

    rospy.loginfo("turntable node is running, start tests")

    while not rospy.is_shutdown():
        # do stuff

        # Get camera info
        rospy.loginfo("###################")
        rospy.loginfo("# Get camera info #")
        rospy.loginfo("###################")
        turntable.camera_info()

        # Check cameras
        rospy.loginfo("####################")
        rospy.loginfo("# Checking cameras #")
        rospy.loginfo("####################")
        turntable.get_images()

        # if complete, wait until shutdown
        if (turntable.complete()):
            rospy.sleep(1)

    rospy.loginfo("Shutting down...")
    cv2.destroyAllWindows()
    



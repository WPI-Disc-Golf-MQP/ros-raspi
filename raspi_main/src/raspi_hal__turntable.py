#!/usr/bin/env python3

from threading import Timer
from typing import Callable

import rospy
from std_msgs.msg import Int8, Float32, Bool

from node_templates import *

import cv2
import os

# cam10degrees = cv2.VideoCapture(1)
cam35degrees = cv2.VideoCapture(4)

class TURNTABLE_STATE(Enum):
    TURNTABLE_IDLE = 0
    RAISING = 1
    SPINNING_PHOTOS = 2
    LOWERING = 3

class hal__turntable(measure_node):
    def __init__(self, completion_callback:Callable[[str], None]=lambda _: None):
        super().__init__(NAME="turntable", STATE_TYPE=TURNTABLE_STATE, 
                         COMPLETION_CALLBACK=completion_callback)

    def state_update(self, msg:Int8):
        self.state = TURNTABLE_STATE(msg.data)
    
    def get_state(self) -> int:
        return self.state.value
    
    def picture_disc(dirname, discName):
        # os.mkdir(dirname)
        # for i in range(0, 10):
        #     # cam10degrees.set(cv2.CAP_PROP_FRAME_WIDTH, 1200)
        #     # cam10degrees.set(cv2.CAP_PROP_FRAME_HEIGHT, 1920)
        #     # cam10degrees.set(cv2.CAP_PROP_FPS, 90)

        #     cam35degrees.set(cv2.CAP_PROP_FRAME_WIDTH, 1200)
        #     cam35degrees.set(cv2.CAP_PROP_FRAME_HEIGHT, 1920)
        #     cam35degrees.set(cv2.CAP_PROP_FPS, 90)

        #     # ret, image = cam10degrees.read()
        #     success, image2 = cam35degrees.read()

        #     # name10Degrees = str(10) + ' ' + discName + str(i+1) + '.jpg'
        #     name35Degrees = str(35) + ' ' + discName + str(i+1) + '.jpg'

        #     # cv2.imwrite(os.path.join(dirname, name10Degrees), img=image)
        #     cv2.imwrite(os.path.join(dirname, name35Degrees), img=image2)

        #     rospy.loginfo(os.path.join(dirname, name35Degrees))
        #     cv2.waitKey(200)

        # # cam10degrees.release()
        # cam35degrees.release()

        if cam35degrees.isOpened():
            cam35degrees.set(cv2.CAP_PROP_FRAME_WIDTH, 1200)
            cam35degrees.set(cv2.CAP_PROP_FRAME_HEIGHT, 1920)
            cam35degrees.set(cv2.CAP_PROP_FPS, 90)
            ret, image = cam35degrees.read()
            cv2.imshow("image", image)
            cv2.waitKey(10000)

            cam35degrees.release()



        # rospy.loginfo("should be saving images")
        cv2.destroyAllWindows()

        

if __name__ == '__main__':
    rospy.init_node('hal__turntable')
    rospy.loginfo("hal__turntable node started")

    def _completion_callback(_):
        print("* Movement Complete, State: " + str(turntable.get_state()) + ", notified via callback.")

    turntable = hal__turntable(_completion_callback)

    hal__turntable.picture_disc("PhotosPls", "test")
    rospy.loginfo("in main loop for turntable")
    while not rospy.is_shutdown():
        # if(turntable.complete() == False):
            # if(turntable.get_state() == 2):
            #     turntable.picture_disc("Testing", "testDisc")
            #     rospy.loginfo("Pictured disc!")
        if(turntable.complete()):
            rospy.sleep(1)
    



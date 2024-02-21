#!/usr/bin/env python3

from typing import Callable

import rospy
from std_msgs.msg import Int8, Float32, Bool, Empty

from node_templates import *

class HEIGHT_STATE(Enum):
    FLEX_IDLE = 0
    FLEX_MEASURING_BOTTOM = 1
    FLEX_MEASURING_TOP = 2
    FLEX_ERROR = -1

class hal_height(motion_node):
    def __init__(self, completion_callback:Callable[[str], None]):
        super().__init__(NAME="height", STATE_TYPE=HEIGHT_STATE, COMPLETION_CALLBACK=completion_callback)
        self._height_sub = rospy.Subscriber('height', Int8, self._height_callback)
        self._height_value:float= -0.0
    
    def _height_callback(self, msg:Int8):
        self._height_value = msg.data
    
    def get_height(self) -> float:
        return self._height_value
    
if __name__ == '__main__':
    rospy.init_node('hal_height')
    rospy.loginfo("hal_height node started")

    def _completion_callback(_):
        print("* Movement Complete, State: " + str(_hal_height.get_state()) + ", notified via callback.")

    _hal_height = hal_height(_completion_callback)

    while not rospy.is_shutdown():
        if (_hal_height.complete()):
            rospy.sleep(1)
            _hal_height.start()
            print("New Motion Started")
        print("+ Node Online" if _hal_height.is_online() else "- Node Offline")
        print("+ Motion Complete" if _hal_height.complete() else "- Measurement Incomplete")
        print("| State: " + str(_hal_height.get_state()))
        rospy.sleep(1)
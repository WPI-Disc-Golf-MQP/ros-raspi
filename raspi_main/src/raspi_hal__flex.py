#!/usr/bin/env python3

from typing import Callable

import rospy
from std_msgs.msg import Int8, Float32, Bool, Empty

from node_templates import *

class FLEX_STATE(Enum):
    FLEX_IDLE = 0
    FLEX_RAISING = 1
    FLEX_LOWERING = 2
    
FLEX_TOPIC = ('flex', Int8,)

class hal_flex(motion_node):
    def __init__(self, completion_callback:Callable[[str], None]):
        super().__init__(NAME="flex", STATE_TYPE=FLEX_STATE, COMPLETION_CALLBACK=completion_callback)
        self._flex_sub = rospy.Subscriber(*FLEX_TOPIC, self._flex_callback)
        self._flex_value:float= -0.0
    
    def _flex_callback(self, msg:Int8):
        self._flex_value = msg.data
    
    def get_flex(self) -> float:
        return self._flex_value
    
if __name__ == '__main__':
    rospy.init_node('hal_flex')
    rospy.loginfo("hal_flex node started")

    def _completion_callback(_):
        print("* Movement Complete, State: " + str(_hal_flex.get_state()) + ", notified via callback.")

    _hal_flex = hal_flex(_completion_callback)

    while not rospy.is_shutdown():
        if (_hal_flex.complete()):
            rospy.sleep(1)
            _hal_flex.start()
            print("New Motion Started")
        print("+ Node Online" if _hal_flex.is_online() else "- Node Offline")
        print("+ Motion Complete" if _hal_flex.complete() else "- Measurement Incomplete")
        print("| State: " + str(_hal_flex.get_state()))
        rospy.sleep(1)
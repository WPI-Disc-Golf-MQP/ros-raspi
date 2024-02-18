#!/usr/bin/env python3

from threading import Timer
from typing import Callable

import rospy
from std_msgs.msg import Int8, Float32, Bool

from node_templates import *

FEEDBACK_TOPIC=("module_a_feedback__turntable_state", Int8)  # bool

class TURNTABLE_STATE(Enum):
    TURNTABLE_IDLE = 0
    RAISING = 1
    SPINNING_PHOTOS = 2
    LOWERING = 3

class hal__turntable(measure_node):
    def __init__(self, completion_callback:Callable[[str], None]=lambda _: None):
        super().__init__(NAME="module_a", COMPLETION_CALLBACK=completion_callback)
        self.state_sub = rospy.Subscriber(*FEEDBACK_TOPIC, self.state_update)
        self.state:TURNTABLE_STATE = TURNTABLE_STATE.TURNTABLE_IDLE

    def state_update(self, msg:Int8):
        self.state = TURNTABLE_STATE(msg.data)
    
    def get_state(self) -> int:
        return self.state.value
        

if __name__ == '__main__':
    rospy.init_node('hal__turntable')
    rospy.loginfo("hal__turntable node started")


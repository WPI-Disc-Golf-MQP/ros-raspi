#!/usr/bin/env python3

from typing import Callable

import rospy
from std_msgs.msg import Int8, Float32, Bool, Empty

from node_templates import *

class LABEL_TAMPER_STATE(Enum):
    LABELER_IDLE = 0
    LABELER_PRINTING = 1
    LABELER_TAMPING = 2
    LABELER_RETURNING= 3
    LABELER_ERROR = -1

class hal__label_tamper(measure_node):
    def __init__(self, completion_callback:Callable[[str], None]):
        super().__init__(NAME="label_tamper", STATE_TYPE=LABEL_TAMPER_STATE, COMPLETION_CALLBACK=completion_callback)
    
if __name__ == '__main__':
    rospy.init_node('hal__label_tamper')
    rospy.loginfo("hal__label_tamper node started")

    def _completion_callback(_):
        print("* Movement Complete, State: " + str(_hal__label_tamper.get_state()) + ", notified via callback.")

    _hal__label_tamper = hal__label_tamper(_completion_callback)

    while not rospy.is_shutdown():
        if (_hal__label_tamper.complete()):
            rospy.sleep(1)
            _hal__label_tamper.start()
            print("New Motion Started")
        print("+ Node Online" if _hal__label_tamper.is_online() else "- Node Offline")
        print("+ Motion Complete" if _hal__label_tamper.complete() else "- Motion Incomplete")
        print("| State: " + str(_hal__label_tamper.get_state()))
        rospy.sleep(1)
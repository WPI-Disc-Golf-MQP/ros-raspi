#!/usr/bin/env python3

from typing import Callable

import rospy
from std_msgs.msg import Int8, Float32, Bool, Empty

from node_templates import *

class LABELER_STATE(Enum):
    LABELER_IDLE = 0
    LABELER_PRINTING = 1
    LABELER_TAMPING = 2
    LABELER_RETURNING= 3
    LABELER_ERROR = -1

class hal__labeler(measure_node):
    def __init__(self, completion_callback:Callable[[str], None]):
        super().__init__(NAME="labeler", STATE_TYPE=LABELER_STATE, COMPLETION_CALLBACK=completion_callback)
    
if __name__ == '__main__':
    rospy.init_node('hal__labeler')
    rospy.loginfo("hal__labeler node started")

    def _completion_callback(_):
        print("* Movement Complete, State: " + str(_hal__labeler.get_state()) + ", notified via callback.")

    _hal__labeler = hal__labeler(_completion_callback)

    while not rospy.is_shutdown():
        if (_hal__labeler.complete()):
            rospy.sleep(1)
            _hal__labeler.start()
            print("New Motion Started")
        print("+ Node Online" if _hal__labeler.is_online() else "- Node Offline")
        print("+ Motion Complete" if _hal__labeler.complete() else "- Measurement Incomplete")
        print("| State: " + str(_hal__labeler.get_state()))
        rospy.sleep(1)
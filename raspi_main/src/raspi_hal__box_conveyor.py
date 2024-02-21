#!/usr/bin/env python3

from typing import Callable

import rospy
from std_msgs.msg import Int8, Float32, Bool, Empty

from node_templates import *

class BOX_CONVEYOR_STATE(Enum):
    BOX_CONVEYOR_IDLE = 0
    BOX_CONVEYOR_ALIGNING = 1
    BOX_CONVEYOR_ADVANCING = 2
    BOX_CONVEYOR_ERROR = -1

class hal_box_conveyor(motion_node):
    def __init__(self, completion_callback:Callable[[str], None]):
        super().__init__(NAME="box_conveyor", STATE_TYPE=BOX_CONVEYOR_STATE, COMPLETION_CALLBACK=completion_callback)
    
if __name__ == '__main__':
    rospy.init_node('hal_box_conveyor')
    rospy.loginfo("hal_box_conveyor node started")

    def _completion_callback(_):
        print("* Movement Complete, State: " + str(_hal_box_conveyor.get_state()) + ", notified via callback.")

    _hal_box_conveyor = hal_box_conveyor(_completion_callback)

    while not rospy.is_shutdown():
        if (_hal_box_conveyor.complete()):
            rospy.sleep(1)
            _hal_box_conveyor.start()
            print("New Motion Started")
        print("+ Node Online" if _hal_box_conveyor.is_online() else "- Node Offline")
        print("+ Motion Complete" if _hal_box_conveyor.complete() else "- Measurement Incomplete")
        print("| State: " + str(_hal_box_conveyor.get_state()))
        rospy.sleep(1)
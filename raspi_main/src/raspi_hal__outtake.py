#!/usr/bin/env python3

from typing import Callable

import rospy
from std_msgs.msg import Int8, Float32, Bool, Empty

from node_templates import *

class OUTTAKE_STATE(Enum):
    OUTTAKE_IDLE = 0
    OUTTAKE_ALIGNING = 1
    OUTTAKE_PREPARING = 2
    OUTTAKE_RELEASING = 3



class hal__outtake(motion_node):
    def __init__(self, completion_callback:Callable[[str], None], labeler_callback:Callable[[None], None] = lambda: None):
        super().__init__(NAME="outtake", STATE_TYPE=OUTTAKE_STATE, 
                         STATE_CHANGE_CALLBACK=self.state_change, COMPLETION_CALLBACK=completion_callback)
        self.labeler_callback = labeler_callback
    
    def state_change(self, old:int, new:int):
        if OUTTAKE_STATE(old) == OUTTAKE_STATE.OUTTAKE_ALIGNING and \
           OUTTAKE_STATE(new) == OUTTAKE_STATE.OUTTAKE_PREPARING: 
            self.labeler_callback()
    
if __name__ == '__main__':
    rospy.init_node('hal__outtake')
    rospy.loginfo("hal__outtake node started")

    def _completion_callback(_):
        print("* Movement Complete, State: " + str(_hal__outtake.get_state()) + ", notified via callback.")

    _hal__outtake = hal__outtake(_completion_callback)

    while not rospy.is_shutdown():
        if (_hal__outtake.complete()):
            rospy.sleep(1)
            _hal__outtake.start()
            print("New Motion Started")
        print("+ Node Online" if _hal__outtake.is_online() else "- Node Offline")
        print("+ Motion Complete" if _hal__outtake.complete() else "- Measurement Incomplete")
        print("| State: " + str(_hal__outtake.get_state()))
        rospy.sleep(1)
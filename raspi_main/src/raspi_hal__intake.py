#!/usr/bin/env python3

from typing import Callable

import rospy
from std_msgs.msg import Int8, Float32, Bool, Empty

from node_templates import *

class INTAKE_STATE(Enum):
    INTAKE_IDLE = 0
    INTAKE_SENDING = 1
    INTAKE_RECIEVING = 3



class hal__intake(motion_node):
    def __init__(self, completion_callback:Callable[[str], None], start_conveyor_callback:Callable[[None], None]):
        super().__init__(NAME="intake", STATE_TYPE=INTAKE_STATE, 
                         STATE_CHANGE_CALLBACK=self.state_change, COMPLETION_CALLBACK=completion_callback)
        self.start_conveyor_callback = start_conveyor_callback
    
    def state_change(self, old:int, new:int):
        if INTAKE_STATE(old) == INTAKE_STATE.INTAKE_SENDING: 
            # If we finished sending disc to conveyor, start conveyor again
            self.start_conveyor_callback()
    
if __name__ == '__main__':
    rospy.init_node('hal__intake')
    rospy.loginfo("hal__intake node started")

    def _completion_callback(_):
        print("* Movement Complete, State: " + str(_hal__intake.get_state()) + ", notified via callback.")

    _hal__intake = hal__intake(_completion_callback)

    while not rospy.is_shutdown():
        if (_hal__intake.complete()):
            rospy.sleep(1)
            _hal__intake.start()
            print("New Motion Started")
        print("+ Node Online" if _hal__intake.is_online() else "- Node Offline")
        print("+ Motion Complete" if _hal__intake.complete() else "- Measurement Incomplete")
        print("| State: " + str(_hal__intake.get_state()))
        rospy.sleep(1)
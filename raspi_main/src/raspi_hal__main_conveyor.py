#!/usr/bin/env python3

from typing import Callable

import rospy
from std_msgs.msg import Int8, Float32, Bool, Empty

from node_templates import *

class CONVEYOR_STATE(Enum):
    CONVEYOR_IDLE = 0
    ADVANCING_TO_NEXT_DISC_EDGE = 1
    WAITING_FOR_INTAKE = 2
    MOVING_TO_CENTER = 3
    BACKUP = 4

class hal__main_conveyor(motion_node):
    def __init__(self, completion_callback:Callable[[str], None]=lambda _: None,
                 ready_for_intake_callback:Callable[[None], None]=lambda: None):
        super().__init__(NAME="main_conveyor", STATE_TYPE=CONVEYOR_STATE, 
                         COMPLETION_CALLBACK=completion_callback, STATE_CHANGE_CALLBACK=self.state_change)
        self.ready_for_intake_callback = ready_for_intake_callback
    
    def state_change(self, old:int, new:int):
        rospy.loginfo("main conveyor made it to state change from {} to {}".format(old, new))
        if CONVEYOR_STATE(new) == CONVEYOR_STATE.WAITING_FOR_INTAKE:
            # If we start waiting for intake, notify the intake to start

            self.ready_for_intake_callback()
    
if __name__ == '__main__':
    rospy.init_node('hal__main_conveyor')
    rospy.loginfo("hal__main_conveyor node started")

    def _completion_callback(_):
        print("* Movement Complete, State: " + str(_hal__main_conveyor.get_state()) + ", notified via callback.")

    _hal__main_conveyor = hal__main_conveyor(_completion_callback)

    while not rospy.is_shutdown():
        if (_hal__main_conveyor.complete()):
            rospy.sleep(1)
            _hal__main_conveyor.start()
            print("New Motion Started")
        print("+ Node Online" if _hal__main_conveyor.is_online() else "- Node Offline")
        print("+ Motion Complete" if _hal__main_conveyor.complete() else "- Measurement Incomplete")
        print("| State: " + str(_hal__main_conveyor.get_state()))
        rospy.sleep(1)
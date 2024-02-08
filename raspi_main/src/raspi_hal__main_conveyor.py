#!/usr/bin/env python3

from typing import Callable

import rospy
from std_msgs.msg import Int8, Float32, Bool

from node_templates import *

class CONVEYOR_STATE(Enum):
    CONVEYOR_IDLE = 0
    MOVING_TO_CENTER = 1
    MOVING_TO_NEXT_DISC = 2
    BACKUP = 3

FEEDBACK_TOPIC=("module_b_feedback__conveyor", Int8)  # bool


class hal__main_conveyor(motion_node):
    def __init__(self, completion_callback:Callable[[str], None]=lambda _: None):
        super().__init__(NAME="main_conveyor", COMPLETION_CALLBACK=completion_callback)
        self.state_sub = rospy.Subscriber(*FEEDBACK_TOPIC, self.state_update)
        self.state:CONVEYOR_STATE = CONVEYOR_STATE.CONVEYOR_IDLE

    def state_update(self, msg:Int8):
        self.state = CONVEYOR_STATE(msg.data)
    
    def get(self) -> int:
        return self.state.value
    
if __name__ == '__main__':
    rospy.init_node('hal__main_conveyor')
    rospy.loginfo("hal__main_conveyor node started")

    def _completion_callback(_):
        print("* Movement Complete, State: " + str(_hal__main_conveyor.get()) + ", notified via callback.")

    _hal__main_conveyor = hal__main_conveyor(_completion_callback)

    while not rospy.is_shutdown():
        if (_hal__main_conveyor.complete()):
            rospy.sleep(1)
            _hal__main_conveyor.start()
            print("New Motion Started")
        print("+ Node Online" if _hal__main_conveyor.is_online() else "- Node Offline")
        print("+ Motion Complete" if _hal__main_conveyor.complete() else "- Measurement Incomplete")
        print("| State: " + str(_hal__main_conveyor.get()))
        rospy.sleep(1)
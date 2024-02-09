#!/usr/bin/env python3

from typing import Callable

import rospy
from std_msgs.msg import Int8, Float32, Bool

from node_templates import *

class INTAKE_STATE(Enum):
    INTAKE_IDLE = 0
    INTAKE_SENDING = 1
    INTAKE_WAITING_FOR_DISC = 2
    INTAKE_RECIEVING = 3

FEEDBACK_TOPIC=("module_b_feedback__intake_state", Int8)  # bool


class hal__intake(motion_node):
    def __init__(self, completion_callback:Callable[[str], None]=lambda _: None):
        super().__init__(NAME="module_b", COMPLETION_CALLBACK=completion_callback)
        self.state_sub = rospy.Subscriber(*FEEDBACK_TOPIC, self.state_update)
        self.state:INTAKE_STATE = INTAKE_STATE.INTAKE_IDLE

    def state_update(self, msg:Int8):
        self.state = INTAKE_STATE(msg.data)
    
    def get_state(self) -> int:
        return self.state.value
    
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
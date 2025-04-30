from typing import Callable

import rospy
from std_msgs.msg import Int8, Float32, Bool, Empty

from node_templates import *

class BACKING_STATE(Enum):
    BACKING_IDLE = 0
    BACKING_MOVE = 1

class hal_backing(motion_node):
    def __init__(self, completion_callback:Callable[[str], None]):
        super().__init__(NAME="backing", STATE_TYPE=BACKING_STATE, COMPLETION_CALLBACK=completion_callback)
    
    def state_update(self, msg:Int8):
        self.state = BACKING_STATE(msg.data)

if __name__ == '__main__':
    rospy.init_node('hal_backing')
    rospy.loginfo("hal_backing node started")

    def _completion_callback(_):
        print("* Movement Complete, State: " + str(_hal_backing.get_state()) + ", notified via callback.")

    _hal_backing = hal_backing(_completion_callback)

    while not rospy.is_shutdown():
        if (_hal_backing.complete()):
            rospy.sleep(1)
            _hal_backing.start()
            print("New Motion Started")
        print("+ Node Online" if _hal_backing.is_online() else "- Node Offline")
        print("+ Motion Complete" if _hal_backing.complete() else "- Measurement Incomplete")
        print("| State: " + str(_hal_backing.get_state()))
        rospy.sleep(1)
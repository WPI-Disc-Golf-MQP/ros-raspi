#!/usr/bin/env python3

from typing import Callable

import rospy
from std_msgs.msg import Int8, Float32, Bool

from node_templates import *

FEEDBACK_TOPIC=("main_conveyor_feedback__distance", Float32)  # bool

MEASUREMENT_DURATION = 1.5 # seconds



class hal__main_conveyor(motion_node):
    def __init__(self, completion_callback:Callable[[str], None]=lambda _: None):
        super().__init__(NAME="main_conveyor", COMPLETION_CALLBACK=completion_callback)
        self.distance_sub = rospy.Subscriber(*FEEDBACK_TOPIC, self.distance_update)
        self.distance = 0

    def distance_update(self, msg:Float32):
        self.distance = msg.data
    
    def get(self) -> float:
        return self.distance
    
if __name__ == '__main__':
    rospy.init_node('hal__main_conveyor')
    rospy.loginfo("hal__main_conveyor node started")

    def _completion_callback(_):
        print("* Movement Complete, Distance: " + str(_hal__main_conveyor.get()) + "mm, notified via callback.")

    _hal__main_conveyor = hal__main_conveyor(_completion_callback)

    while not rospy.is_shutdown():
        if (_hal__main_conveyor.complete()):
            _hal__main_conveyor.start()
            print("New Motion Started")
        print("+ Node Online" if _hal__main_conveyor.is_online() else "- Node Offline")
        print("+ Motion Complete" if _hal__main_conveyor.complete() else "- Measurement Incomplete")
        print("| Distance: " + str(_hal__main_conveyor.get()) + "mm")
        rospy.sleep(1)
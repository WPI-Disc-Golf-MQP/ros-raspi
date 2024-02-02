#!/usr/bin/env python3

from threading import Timer
from typing import Callable

import rospy
from std_msgs.msg import Int8, Float32, Bool

from node_templates import *

FEEDBACK_TOPIC=("scale_feedback__weight", Float32)  # bool

MEASUREMENT_DURATION = 1.5 # seconds
DISC_PRESENCE_THRESHOLD = 100 # TODO: Set to lower value in grams

# TODO: Add tare logic on microcontroller; tare when no disc is present
# TODO: Add power button logic on microcontroller; always stay on



class hal__scale(measure_node):
    def __init__(self, completion_callback:Callable[[str], None]=lambda _: None):
        super().__init__(NAME="scale", COMPLETION_CALLBACK=completion_callback)
        self.weight_sub = rospy.Subscriber(*FEEDBACK_TOPIC, self.weight_update)
        self.weight = -100000
    
    def valid_measurement(self) -> bool:
        return self.weight > DISC_PRESENCE_THRESHOLD

    def weight_update(self, msg:Float32):
        self.weight = msg.data
    
    def get(self) -> float:
        return self.weight
        
    def complete(self) -> bool:
        return super().complete() and self.valid_measurement()
    
if __name__ == '__main__':
    rospy.init_node('hal__scale')
    rospy.loginfo("hal__scale node started")

    def _completion_callback(_):
        print("* Measurement Complete, Weight: " + str(_hal__scale.get()) + "g, notified via callback.")

    _hal__scale = hal__scale(_completion_callback)

    while not rospy.is_shutdown():
        if (_hal__scale.complete()):
            _hal__scale.start()
            print("New Measurement Started for duration " + str(MEASUREMENT_DURATION) + "s")
        print("+ Node Online" if _hal__scale.is_online() else "- Node Offline")
        print("+ Measurement Complete" if _hal__scale.complete() else "- Measurement Incomplete")
        print("| Weight: " + str(_hal__scale.get()) + "g")
        rospy.sleep(1)
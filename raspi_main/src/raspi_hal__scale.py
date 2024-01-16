#!/usr/bin/env python3

from threading import Timer
from typing import Callable

import rospy
from std_msgs.msg import Int8, Float32, Bool

from node_templates import serial_node, measure_node

FEEDBACK_TOPIC=("scale_feedback__weight", Float32)  # bool
REQUEST_TOPIC=("scale_request", Int8) # use REQUEST enum

MEASUREMENT_DURATION = 1.5 # seconds
DISC_PRESENCE_THRESHOLD = 100 # TODO: Set to lower value in grams

# TODO: Add tare logic on microcontroller; tare when no disc is present
# TODO: Add power button logic on microcontroller; always stay on



class hal__scale(serial_node, measure_node):
    def __init__(self, completion_callback:Callable[[float], None]):
        super().__init__("scale")
        self.weight_sub = rospy.Subscriber(*FEEDBACK_TOPIC, self.weight_update)
        self.weight = -100000
        self.new_timer = lambda : Timer(MEASUREMENT_DURATION, self._completion_callback)
        self.timer = self.new_timer()
        self.completion_callback = completion_callback

    def weight_update(self, msg:Float32):
        self.weight = msg.data
        if self.weight < DISC_PRESENCE_THRESHOLD:
            self.timer = self.new_timer()
    
    def _completion_callback(self):
        self.completion_callback(self.get())
    
    def get(self) -> float:
        return self.weight
    
    def start(self):
        self.timer = self.new_timer()
        self.timer.start()       
        
    def complete(self) -> bool: # intialized and node online and measurement complete
        return self.weight != -100000 and self.is_online() and not self.timer.is_alive()
    
    def can_start_measurement(self) -> bool:
        return self.is_online()
    
if __name__ == '__main__':
    rospy.init_node('hal__scale')
    rospy.loginfo("hal__scale node started")

    def _completion_callback(weight:float):
        print("* Measurement Complete, Weight: " + str(weight) + "g, notified via callback.")

    _hal__scale = hal__scale(_completion_callback)

    while not rospy.is_shutdown():
        if (_hal__scale.complete()):
            _hal__scale.start()
            print("New Measurement Started for duration " + str(MEASUREMENT_DURATION) + "s")
        print("+ Node Online" if _hal__scale.is_online() else "- Node Offline")
        print("+ Measurement Complete" if _hal__scale.complete() else "- Measurement Incomplete")
        print("| Weight: " + str(_hal__scale.get()) + "g")
        rospy.sleep(1)
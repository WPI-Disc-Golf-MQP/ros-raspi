#!/usr/bin/env python3

from std_msgs.msg import Bool, Int8

import rospy

from node_templates import serial_node, NODE_STATUS, REQUEST

FEEDBACK_TOPIC=("led_feedback", Bool)
REQUEST_TOPIC=("led_request", Int8) # use REQUEST enum

class hal__led_demo(serial_node):
    def __init__(self):
        super().__init__(NAME:="led")
        self.request_pub = rospy.Publisher(*REQUEST_TOPIC, queue_size=10)
        self.feedback_sub = rospy.Subscriber(*FEEDBACK_TOPIC, self.feedback_callback)
        self.led_status = False
        self.request_pub.publish(REQUEST.INITIALIZING)

    def set_led(self, state: bool):
        self.request_pub.publish((REQUEST.REQUEST__STANDARD if state else REQUEST.ONLINE_IDLE).value)
    
    def feedback_callback(self, msg):
        self.led_status = msg.data

    def get_led(self) -> bool:
        return self.led_status


if __name__ == '__main__':
    rospy.init_node('hal__led_demo')
    rospy.loginfo("hal__led_demo node started")

    _hal__led_demo = hal__led_demo()
    _hal__led_demo.set_online_callback(lambda: rospy.logwarn("[Outside HAL] LED now online"))
    _hal__led_demo.set_offline_callback(lambda: rospy.logerr("[Outside HAL] LED now offline"))

    while not rospy.is_shutdown():
        _hal__led_demo.set_led(True)
        rospy.sleep(0.1)
        print("Node Offline" if not _hal__led_demo.is_online() else "Node Online" + \
            ", LED Status: " + ("On" if _hal__led_demo.get_led() else "Off"))
        rospy.sleep(0.75)

        _hal__led_demo.set_led(False)
        rospy.sleep(0.1)
        print("Node Offline" if not _hal__led_demo.is_online() else "Node Online" + \
            ", LED Status: " + ("On" if _hal__led_demo.get_led() else "Off"))
        rospy.sleep(1.5)
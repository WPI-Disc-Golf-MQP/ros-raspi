#!/usr/bin/env python3

PUB_TOPIC = "led"
WATCHDOG_TOPIC = "nucleo_led__feedback"

import rospy
from std_msgs.msg import Bool
from node_templates import serial_node

class hal__led_demo(serial_node):
    def __init__(self):
        self.led_pub = rospy.Publisher(PUB_TOPIC, Bool, queue_size=10)
        super().__init__(watchdog_sub_topic=WATCHDOG_TOPIC, watchdog_sub_type=Bool)

    def set_led(self, state: bool):
        self.led_pub.publish(state)


if __name__ == '__main__':
    rospy.init_node('hal__led_demo')
    rospy.loginfo("hal__led_demo node started")

    _hal__led_demo = hal__led_demo()

    while not rospy.is_shutdown():
        _hal__led_demo.set_led(True)
        print("Node Online" if _hal__led_demo.online() else "Node Offline")
        rospy.sleep(1)
        _hal__led_demo.set_led(False)
        print("Node Online" if _hal__led_demo.online() else "Node Offline")
        rospy.sleep(1)
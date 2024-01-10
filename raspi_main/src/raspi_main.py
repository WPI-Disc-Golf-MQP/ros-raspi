#!/usr/bin/env python3

#from raspi_ui import ui

import rospy
from std_msgs.msg import String

class raspi_main:
    def __init__(self):
        rospy.init_node('raspi_main')
        rospy.loginfo("raspi_main node started")

        # --- Subscribers ---

        # --- Publishers ---

        self.button_b_subscriber = rospy.Subscriber('button_b', String, self.button_b_callback)

    def button_b_callback(self, msg):
        print("[main]Button B Pressed")
    
    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.spin()

if __name__ == '__main__':
    raspi_main = raspi_main()
    raspi_main.run()
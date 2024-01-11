#!/usr/bin/env python3

#from raspi_ui import ui

import rospy
from std_msgs.msg import String

class raspi_main:
    def __init__(self):
        rospy.init_node('raspi_main')
        rospy.loginfo("raspi_main node started")

        # --- Subscribers ---

        self.button_b_subscriber = rospy.Subscriber('button_b', String, self.button_b_callback)
        self.nucleo_response_subscriber = rospy.Subscriber('nucleo_response', String, self.nucleo_response_callback)

        # --- Publishers ---

        self.nucleo_ping_publisher = rospy.Publisher('nucleo_ping', String, queue_size=10)

    def button_b_callback(self, msg):
        print("[main] Button B Pressed")
        self.nucleo_ping_publisher.publish("a")
    
    def nucleo_response_callback(self, msg):
        print("[main] Nucleo Response Received: " + msg.data)
    
    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.spin()

if __name__ == '__main__':
    main = raspi_main()
    main.run()
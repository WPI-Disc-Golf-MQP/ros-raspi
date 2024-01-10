#!/usr/bin/env python3

import ui

import rospy

class raspi_main:
    def __init__(self):
        rospy.init_node('raspi_main')
        rospy.loginfo("raspi_main node started")
        
        self.ui = raspi_ui.ui()

        # --- Subscribers ---

        # --- Publishers ---
    
    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.spin()

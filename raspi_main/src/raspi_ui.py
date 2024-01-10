#!/usr/bin/env python3

import rospy
import customtkinter as ctk
import tkinter as tk
from tkinter import ttk

from std_msgs.msg import String

class ui:
    def __init__(self):
        rospy.init_node('raspi_ui')
        rospy.loginfo("raspi_ui node started")

        #ros_service_test = rospy.ServiceProxy('ui_service_test', )
        self.ros_publish_test = rospy.Publisher('button_b', String, queue_size=10)
        self.ros_publish_test.publish("Hello World")

        ctk.set_appearance_mode("Light")
        #ctk.set_default_color_theme("blue")

        self.app = ctk.CTk()
        self.app.title("ROS with Nucleo Demo")

        style = ttk.Style()
        style.theme_use("clam")

        main_frame  = ctk.CTkFrame(self.app)
        main_frame.grid(row=0,  column=0, columnspan=2, padx=5,pady=5)

        def callback__button_b():
            self.ros_publish_test.publish("Hello World")
            print("Button B Pressed")

        title = ctk.CTkLabel(main_frame, text="ROS Nucleo Demo",font=ctk.CTkFont(size=24, weight="bold"))
        title.grid(row=0, column=1)
        #button_b = ctk.CTkButton(main_frame, text="Publish Test", command=lambda : self.ros_publish_test.publish("Hello World"))
        button_b = ctk.CTkButton(main_frame, text="Publish Test", command=callback__button_b)
        button_b.grid(row=1,  column=0)
        #button_c = ctk.CTkButton(main_frame, text="Service Test", command=lambda : ros_service_test.publish("Hello World"))
        #button_c.grid(row=1, column=2)

    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        self.app.mainloop()
        
if __name__ == '__main__':
    ui = ui()
    ui.run()
#!/usr/bin/env python3
ch

import rospy
import customtkinter as ctk
import tkinter as tk
from tkinter import ttk

class ui:
    rospy.init_node('raspi_ui')
    rospy.loginfo("raspi_ui node started")

    ros_publish_test = rospy.Publisher('ui_publish_test', Bool)
    ros_service_test = rospy.Service('ui_service_test', Bool)

    ctk.set_appearance_mode("Light")
    ctk.set_default_color_theme("blue")

    app = ctk.CTk()
    app.title("ROS with Nucleo Demo")

    style = ttk.Style()
    style.theme_use("clam")

    main_frame  = ctk.CTkFrame(app)
    main_frame.grid(row=0,  column=0, columnspan=2, padx=5,pady=5)

    stop_button = ctk.CTkLabel(main_frame, text="ROS Nucleo Demo",font=ctk.CTkFont(size=24, weight="bold"))
    stop_button.grid(row=0, column=1)
    start_button = ctk.CTkButton(main_frame, text="Publish Test", command=lambda : ros_publish_test.publish("Hello World"))
    start_button.grid(row=1,  column=0)
    stop_button = ctk.CTkButton(main_frame, text="Service Test", command=lambda : ros_service_test.publish("Hello World"))
    stop_button.grid(row=1, column=2)

    def __init__(self):
        pass

    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        self.app.mainloop()
        
if __name__ == '__main__':
    ui = ui()
    ui.run()
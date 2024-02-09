#!/usr/bin/env python3

from functools import partial
import rospy
import customtkinter as ctk
import tkinter as tk
from tkinter import ttk
from ui_constants import UIConstants

from std_msgs.msg import String, Empty

class ui:
    def __init__(self):
        rospy.init_node('raspi_ui')
        rospy.loginfo("raspi_ui node started")

        #ros_service_test = rospy.ServiceProxy('ui_service_test', )
        self.ui_pub = rospy.Publisher('ui_button', String, queue_size=10)

        ctk.set_appearance_mode("Light")
        #ctk.set_default_color_theme("blue")

        self.app = ctk.CTk()
        self.app.title("Disc Inventory Demo")

        style = ttk.Style()
        style.theme_use("clam")

        main_frame  = ctk.CTkFrame(self.app)
        main_frame.grid(row=0,  column=1, padx=5,pady=5)

        title = ctk.CTkLabel(main_frame, text="Disc Inventory Demo",font=ctk.CTkFont(size=24, weight="bold"))
        title.grid(row=0, column=0, padx=5, pady=15)
        
        #self.publishers = []
        self.buttons = []
        
        for i in range(len(UIConstants)):
            pub_name = str(UIConstants._member_names_[i])
            #self.publishers.append(rospy.Publisher(pub_name, String, queue_size=10))
            self.buttons.append(button := ctk.CTkButton(main_frame, text=pub_name.replace('_'," "), command=partial(self.ui_pub.publish,pub_name),width=300,height=100,font=ctk.CTkFont(size=24)))
            button.grid(row=int(i/3)+2,  column=i%3, padx=5, pady=5)
        

    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        self.app.mainloop()
        
if __name__ == '__main__':
    _ui = ui()
    _ui.run()
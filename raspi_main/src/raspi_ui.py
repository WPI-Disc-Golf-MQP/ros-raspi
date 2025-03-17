#!/usr/bin/env python3

from functools import partial
import rospy
import customtkinter as ctk
import tkinter as tk
from tkinter import ttk
from ui_constants import ControlButtons, DebuggingButtons

from std_msgs.msg import String, Empty, Float32 


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
        title.grid(row=0, column=1, padx=5, pady=15)
        
        #self.publishers = []
        self.buttons = []
        
        for i in range(len(ControlButtons)):
            pub_name = str(ControlButtons._member_names_[i])
            #self.publishers.append(rospy.Publisher(pub_name, String, queue_size=10))
            self.buttons.append(button := ctk.CTkButton(main_frame, text=pub_name.replace('_'," "), command=partial(self.ui_pub.publish,pub_name),width=300,height=100,font=ctk.CTkFont(size=24)))
            button.grid(row=int(i/3)+2,  column=i%3, padx=5, pady=5)

        self.scale_weight = ctk.CTkLabel(main_frame, text="Weight: 0g",font=ctk.CTkFont(size=24, weight="bold"))
        self.scale_weight.grid(row=6, column=1, padx=5, pady=15)
        self.scale_sub = rospy.Subscriber('scale_feedback__weight', Float32, self.scale_callback)
        
        self.top_conveyor_tracker_readout = ctk.CTkLabel(main_frame, text="",font=ctk.CTkFont(size=24, weight="bold"))
        self.top_conveyor_tracker_readout.grid(row=7, column=1, padx=5, pady=15)
        self.top_conveyortracker_readout_sub = rospy.Subscriber('top_conveyor_tracker', String, self.top_conveyor_tracker_readout_callback)

        self.main_conveyor_tracker_readout = ctk.CTkLabel(main_frame, text="",font=ctk.CTkFont(size=24, weight="bold"))
        self.main_conveyor_tracker_readout.grid(row=8, column=1, padx=5, pady=15)
        self.main_conveyor_tracker_readout_sub = rospy.Subscriber('main_conveyor_tracker', String, self.main_conveyor_tracker_readout_callback)

        for i in range(len(DebuggingButtons)):
            pub_name = str(DebuggingButtons._member_names_[i])
            #self.publishers.append(rospy.Publisher(pub_name, String, queue_size=10))
            self.buttons.append(button := ctk.CTkButton(main_frame, text=pub_name.replace('_'," "), command=partial(self.ui_pub.publish,pub_name),width=300,height=100,font=ctk.CTkFont(size=24)))
            button.grid(row=8+int(i/3)+2,  column=i%3, padx=5, pady=5)

        self.app.mainloop()

    def top_conveyor_tracker_readout_callback(self, msg:String):
        self.top_conveyor_tracker_readout.configure(text=msg.data, require_redraw=True)
        print(msg.data)

    def main_conveyor_tracker_readout_callback(self, msg:String):
        self.main_conveyor_tracker_readout.configure(text=msg.data, require_redraw=True)
        print(msg.data)

    def scale_callback(self, msg:Float32):
        self.scale_weight.configure(text="Weight: " + str(round(msg.data,3)) + "g", require_redraw=True)
        print("Weight: " + str(msg.data) + "g")
        
if __name__ == '__main__':
    _ui = ui()
    
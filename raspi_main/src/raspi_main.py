#!/usr/bin/env python3


import rospy
from std_msgs.msg import String

from raspi_hal__led_demo import hal__led_demo
from raspi_hal__scale import hal__scale
from raspi_hal__main_conveyor import hal__main_conveyor
from raspi_state_machine import raspi_state_machine as state_machine

from disc_record import *
from node_templates import *

class raspi_main:
    def __init__(self):
        rospy.init_node('raspi_main')
        rospy.loginfo("raspi_main node started")

        # --- HAL (Hardware Abstraction Layer) ---

        self.hal__led_demo = hal__led_demo()
        self.hal__scale = hal__scale(self.hal_measure_callback)
        self.hal__main_conveyor = hal__main_conveyor(self.hal_motion_callback)

        self.HALs: dict[str,serial_node] = {'led':self.hal__led_demo, 'scale':self.hal__scale,
                                            'main_conveyor':hal__main_conveyor()}
        self.HALs_motion = filter(lambda HAL: isinstance(HAL, motion_node), self.HALs.values())
        self.HALs_measure = filter(lambda HAL: isinstance(HAL, measure_node), self.HALs.values())

        # --- Subscribers ---

        self.button_b_subscriber = rospy.Subscriber('button_b', String, self.button_b_callback)
        self.nucleo_response_subscriber = rospy.Subscriber('nucleo_response', String, self.nucleo_response_callback)

        # --- Publishers ---

        self.nucleo_ping_publisher = rospy.Publisher('nucleo_ping', String, queue_size=10)

        # --- Discs ---

        self.discs = []
        self.new_disc = lambda : disc_record()
        self.get_disc_by_location = lambda location : next((disc for disc in self.discs if disc.location == location), None)

        # --- State Machine ---

        self.state_machine = state_machine(self.can_move_discs, self.move_discs, self.can_start_measurement, self.start_measurement, self.can_idle)
        self.state = lambda : self.state_machine.current_state
        #self.states = Enum("init", "moving", "measuring", "idle")
        #self.state = self.states.init

    def can_move_discs(self) -> bool:
        return all([motion_hal.can_start_motion() for motion_hal in self.HALs_motion]) \
            and all([measure_hal.complete() for measure_hal in self.HALs_measure])
    
    def move_discs(self):
        for motion_hal in self.HALs_motion:
            motion_hal.start()
    
    def can_start_measurement(self) -> bool:
        return all([measure_hal.can_start_measurement() for measure_hal in self.HALs_measure]) \
            and all([motion_hal.complete() for motion_hal in self.HALs_motion])
    
    def start_measurement(self):
        for measure_hal in self.HALs_measure:
            measure_hal.start()
    
    def can_idle(self) -> bool:
        return False # TODO: Implement checking for when top disc cue is empty

    def hal_measure_callback(self, node_name:str):
        node = self.HALs[node_name]
        rospy.loginfo("* " + node_name " measurement complete, Notified via callback")
        if type(node) == hal__scale and disc := self.get_disc_by_location(location.MAIN_CONVAYOR__SCALE):
            disc.weight = weight
        self.state_machine.cycle()
        
    def hal_motion_callback(self, node_name:str):
        node = self.HALs[node_name]
        rospy.loginfo("* " + node_name " motion complete, Notified via callback")
        self.state_machine.cycle()

    def button_a_callback(self, msg):
        print("[main] Button A Pressed")
        self.start_motion()

    def button_b_callback(self, msg):
        print("[main] Button B Pressed")
        self.nucleo_ping_publisher.publish("a")
        self.hal__scale.start()
    
    
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
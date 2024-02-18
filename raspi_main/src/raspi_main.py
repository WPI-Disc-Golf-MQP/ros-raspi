#!/usr/bin/env python3


from typing import Iterable
import rospy
from std_msgs.msg import String

from raspi_hal__led_demo import hal__led_demo
from raspi_hal__scale import hal__scale
from raspi_hal__main_conveyor import hal__main_conveyor
from raspi_hal__intake import hal__intake

from disc_record import *
from node_templates import *
from ui_constants import UIConstants

class PROCESS_STATE(Enum):
    INIT = 0,
    MEASURING = 1,
    MOVING_OUTTAKE_TO_BOX = 2,
    MOVING_MAIN_CONVEYOR = 3,
    MOVING_INTAKE = 4,
    MOVING_TOP_CONVEYOR = 5,
    IDLE = 6
MOTION_STATES = [PROCESS_STATE.MOVING_OUTTAKE_TO_BOX, PROCESS_STATE.MOVING_MAIN_CONVEYOR, PROCESS_STATE.MOVING_INTAKE, PROCESS_STATE.MOVING_TOP_CONVEYOR]

class raspi_main:
    def __init__(self):
        rospy.init_node('raspi_main')
        rospy.loginfo("raspi_main node started")

        # --- HAL (Hardware Abstraction Layer) ---

        self.hal__led_demo = hal__led_demo()
        self.hal__scale = hal__scale(self.hal_measure_callback)
        self.hal__main_conveyor = hal__main_conveyor(self.hal_motion_callback)
        self.hal__intake = hal__intake(self.hal_motion_callback)

        self.HALs_motion: dict[str,motion_node] = {
            'main_conveyor':self.hal__main_conveyor, 
            'intake':self.hal__intake}
        self.HALs_measure: dict[str,measure_node] = {'scale':self.hal__scale}
        self.HALs: dict[str,serial_node] = {**self.HALs_motion, **self.HALs_measure}

        # --- Subscribers ---

        self.button_b_subscriber = rospy.Subscriber('ui_button', String, self.ui_callback)

        # --- Publishers ---


        # --- Discs ---

        self.discs = []
        self.new_disc = lambda : disc_record()
        self.get_disc_by_location = lambda location : next((disc for disc in self.discs if disc.location == location), None)

        # --- State Machine ---
        
        self.state = PROCESS_STATE.INIT

        #self.state_machine = state_machine(self.can_move_discs, self.move_discs, self.can_start_measurement, self.start_measurement, self.can_idle)
        #self.state = lambda : self.state_machine.current_state
        #self.states = Enum("init", "moving", "measuring", "idle")
        #self.state = self.states.init


    def move_everything(self): ## written by matt for testing the intake module 
        # - move the outtake 
        # - move the conveyor
        self.hal__main_conveyor.start()

        # - move the intake (because we know that)
        self.hal__intake.start() 




    def check_state_transition(self):
        pass
            
    
    def can_move_discs(self) -> bool:
        return all([motion_hal.ready() for motion_hal in self.HALs_motion.values()]) \
            and all([measure_hal.complete() for measure_hal in self.HALs_measure.values()])
    

    # this function does not take into account that the main conveyor has to complete before the intake is called. Replaced with move_everything()
    def move_discs(self):
        for motion_hal in self.HALs_motion.values():
            motion_hal.start()
    
    def can_start_measurement(self) -> bool:
        return all([measure_hal.ready() for measure_hal in self.HALs_measure.values()]) \
            and all([motion_hal.complete() for motion_hal in self.HALs_motion.values()])
    
    def start_measurement(self):
        for measure_hal in self.HALs_measure.values():
            measure_hal.start()
    
    def can_idle(self) -> bool:
        return False # TODO: Implement checking for when top disc cue is empty

    def hal_measure_callback(self, node_name:str):
        try:
            node = self.HALs_measure[node_name]
        except KeyError:
            rospy.logerr("Measurement complete callback received from unknown node: " + node_name)
            return
        rospy.loginfo("* " + node_name + " measurement complete, Notified via callback")
        # if type(node) == hal__scale and (disc := self.get_disc_by_location(location.MAIN_CONVAYOR__SCALE)):
        #     disc.weight = self._hal__scale.get()
        self.check_state_transition()
        
    def hal_motion_callback(self, node_name:str):
        try:
            node = self.HALs_motion[node_name]
        except KeyError:
            rospy.logerr("Motion complete callback received from unknown node: " + node_name)
            return
        rospy.loginfo("* " + node_name + " motion complete, Notified via callback")
        self.check_state_transition()

    def ui_callback(self, btn:String):
        rospy.loginfo("[main] UI Button " + btn.data + " Pressed")
        if btn.data == UIConstants.CONVEYOR_START.name:
            # if not self.can_move_discs():
            #     rospy.logwarn("Cannot move conveyor, not all nodes are ready")
            #     return
            self.hal__main_conveyor.start()
        elif btn.data == UIConstants.INTAKE_START.name:
            # if not self.can_move_discs():
            #     rospy.logwarn("Cannot move intake, not all nodes are ready")
            #     return
            self.hal__intake.start()
        elif btn.data == UIConstants.MEASURE_START.name:
            # if not self.can_start_measurement():
            #     rospy.logwarn("Cannot start measurement, not all nodes are ready")
            #     return
            self.start_measurement()
        elif btn.data == UIConstants.MOTION_START.name:
            # if not self.can_move_discs():
            #     rospy.logwarn("Cannot move discs, not all nodes are ready")
            #     return
            self.move_discs()
        elif btn.data == UIConstants.HOME_ALL.name or btn.data == UIConstants.STOP.name:
            for hal in self.HALs.values():
                hal.request(REQUEST.WAITING)
        elif btn.data == UIConstants.MOVE_EVERYTHING.name: 
            self.move_everything()
    
    
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
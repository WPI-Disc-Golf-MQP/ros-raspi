#!/usr/bin/env python3


from typing import Iterable
from raspi_hal__box_conveyor import hal_box_conveyor
from raspi_hal__flex import hal_flex
from raspi_hal__height import hal_height
from raspi_hal__label_tamper import hal__label_tamper
from raspi_hal__outtake import hal__outtake
from raspi_hal__turntable import hal__turntable
import rospy
from std_msgs.msg import String
import cv2
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
    MOVING_INTAKE_TOP_CONVEYOR = 4,
    IDLE = 5
MOTION_STATES = [PROCESS_STATE.MOVING_OUTTAKE_TO_BOX, PROCESS_STATE.MOVING_MAIN_CONVEYOR, PROCESS_STATE.MOVING_INTAKE_TOP_CONVEYOR]

class raspi_main:
    def __init__(self):
        rospy.init_node('raspi_main')
        rospy.loginfo("raspi_main node started")

        # --- HAL (Hardware Abstraction Layer) ---

        self.hal__scale = hal__scale(self._callback__scale_complete)
        self.hal__flex = hal_flex(self._callback__flex_complete)
        self.hal__height = hal_height(self._callback__height_complete)
        
        self.hal__main_conveyor = hal__main_conveyor(self._callback__main_conveyor_complete, self._callback_main_conveyor_ready_for_intake)
        self.hal__intake = hal__intake(self._callback__intake_complete, self._callback_intake_ready_for_main_conveyor)
        self.hal__outtake = hal__outtake(self._callback__outtake_complete)
        self.hal__turntable = hal__turntable(self._callback__turntable_complete)
        self.hal__labeler = hal__label_tamper(self._callback__label_tamper_complete)
        self.hal__box_conveyor = hal_box_conveyor(self._callback__box_conveyor_complete)

        self.HALs_motion: dict[str,motion_node] = {
            'main_conveyor':self.hal__main_conveyor, 
            'intake':self.hal__intake,
            'outtake':self.hal__outtake,
            'box_conveyor':self.hal__box_conveyor}
        self.HALs_measure: dict[str,measure_node] = {
            'turntable':self.hal__turntable,
            'scale':self.hal__scale,
            'flex':self.hal__flex,
            'height':self.hal__height,
            'labeler':self.hal__labeler}
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


    def start_intake(self):
        print("Should be tarting intake now (TODO but not yet)")
        pass

    def advance(self): ## written by matt for testing the intake module 
        # - move the outtake 
        # - move the conveyor
        # self.hal__main_conveyor.start()
        

        # # - move the intake (because we know that)
        self.hal__intake.start() 
        rospy.loginfo("INTAKE BUTTON PRESSED HOORAY")
        
        # self.hal_turntable.picture_disc("Test", "testDisc")


    def camera10degreees(self):
        self.hal__turntable.picture_disc(cv2.VideoCapture(4))
        rospy.loginfo("should be showing camera :)")
    
    def camera35degrees(self):
        self.hal__turntable.picture_disc(cv2.VideoCapture(0))
    
    def camera90degrees(self):
        self.hal__turntable.picture_disc(cv2.VideoCapture(2))



    def check_state_transition(self):
        #TODO: Needs to be fleshed out.
        if self.state == PROCESS_STATE.INIT:
            if all([hal.is_online() for hal in self.HALs.values()]):
                self.state = PROCESS_STATE.IDLE
                # TODO: Display "initialization complete" on UI
        elif self.state == PROCESS_STATE.MEASURING:
            if all([hal.complete() for hal in self.HALs_measure.values()]):
                self.state = PROCESS_STATE.MOVING_OUTTAKE_TO_BOX
                # TODO: Start outtake process
        elif self.state == PROCESS_STATE.MOVING_INTAKE_TOP_CONVEYOR:
            if all([hal.complete() for hal in (self.hal__intake, self.hal__main_conveyor)]):
                self.state = PROCESS_STATE.MEASURING
                self.start_measurement()
            
            
    
    def can_move_discs(self) -> bool:
        return all([motion_hal.ready() for motion_hal in self.HALs_motion.values()]) \
            and all([measure_hal.complete() for measure_hal in self.HALs_measure.values()])
    

    # this function does not take into account that the main conveyor has to complete before the intake is called. Replaced with move_everything()
    def move_discs(self):

        self.hal__main_conveyor.start()

        # for motion_hal in self.HALs_motion.values():
        #     motion_hal.start()
    
    def can_start_measurement(self) -> bool:
        rospy.loginfo("starting something")
        return all([measure_hal.ready() for measure_hal in self.HALs_measure.values()]) \
            and all([motion_hal.complete() for motion_hal in self.HALs_motion.values()])

    
    def start_measurement(self):
        for measure_hal in self.HALs_measure.values():
            measure_hal.start()
    
    def queue_complete(self) -> bool:
        return False # TODO: Implement checking for when top disc conveyor is empty
    
    

    def _callback__scale_complete(self, node_name:str):
        rospy.loginfo("* SCALE measurement complete, Notified via callback")
        if disc := self.get_disc_by_location(location.MAIN_CONVAYOR__SCALE):
            disc.weight = self._hal__scale.get_weight()
        self.check_state_transition()
    
    def _callback__flex_complete(self, node_name:str):
        rospy.loginfo("* FLEX measurement complete, Notified via callback")
        if disc := self.get_disc_by_location(location.MAIN_CONVAYOR__FLEXIBILITY):
            disc.flex = self._hal__flex.get_flex()
        self.check_state_transition()
    
    def _callback__height_complete(self, node_name:str):
        rospy.loginfo("* HEIGHT measurement complete, Notified via callback")
        if disc := self.get_disc_by_location(location.MAIN_CONVAYOR__FLEXIBILITY):
            disc.height = self._hal__height.get_height()
        self.check_state_transition()
    
    def _callback__main_conveyor_complete(self, node_name:str):
        rospy.loginfo("* MAIN CONVEYOR motion complete, Notified via callback")
        # TODO: Implement moving disc records to next location
        self.check_state_transition()
    
    def _callback__intake_complete(self, node_name:str):
        rospy.loginfo("* INTAKE motion complete, Notified via callback")
        self.check_state_transition()
        
    def _callback__turntable_complete(self, node_name:str):
        rospy.loginfo("* TURNTABLE motion complete, Notified via callback")
        self.check_state_transition()
    
    def _callback__outtake_complete(self, node_name:str):
        rospy.loginfo("* OUTTAKE motion complete, Notified via callback")
        self.check_state_transition()
    
    def _callback__label_tamper_complete(self, node_name:str):
        rospy.loginfo("* LABELER motion complete, Notified via callback")
        self.check_state_transition()
    
    def _callback__box_conveyor_complete(self, node_name:str):
        rospy.loginfo("* BOX CONVEYOR motion complete, Notified via callback")
        self.check_state_transition()
        
    def _callback_intake_ready_for_main_conveyor(self):
        #if self.state == PROCESS_STATE.MOVING_INTAKE_TOP_CONVEYOR:
            rospy.loginfo("* INTAKE ready for main conveyor, Notified via callback")
            self.hal__main_conveyor.start()
            # No state change since main conveyor is only finishing motion (with centering)
            
    def _callback_main_conveyor_ready_for_intake(self):
        #if self.state == PROCESS_STATE.MOVING_MAIN_CONVEYOR:
            rospy.loginfo("* MAIN CONVEYOR ready for intake, Notified via callback")
            self.hal__intake.start()
            self.state = PROCESS_STATE.MOVING_INTAKE_TOP_CONVEYOR
        
        

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
            pass
            # self.move_discs()
        elif btn.data == UIConstants.TURNTABLE_START.name:
            self.hal__turntable.start() 
             
        elif btn.data == UIConstants.HOME_ALL.name or btn.data == UIConstants.STOP.name:
            for hal in self.HALs.values():
                hal.request(REQUEST.STOP)
        elif btn.data == UIConstants.ADVANCE.name: 
            self.advance()
        
        elif btn.data == UIConstants.CAMERA10DEGREES.name:
            self.camera10degreees()

        elif btn.data == UIConstants.CAMERA35DEGREES.name:
            self.camera35degrees()
        
        elif btn.data == UIConstants.CAMERA90DEGREES.name:
            self.camera90degrees()
    
    
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
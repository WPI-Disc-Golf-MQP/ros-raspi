#!/usr/bin/env python3


##### OBSOLETE - use __main__ code in hal__turntable instead

#from typing import Iterable
# from raspi_hal__box_conveyor import hal_box_conveyor
# from raspi_hal__flex import hal_flex
# from raspi_hal__height import hal_height
# from raspi_hal__label_tamper import hal__label_tamper
# from raspi_hal__outtake import hal__outtake
# from raspi_hal__turntable import hal__turntable
# import rospy
# from std_msgs.msg import String
# import cv2
# from raspi_hal__scale import hal__scale
# from raspi_hal__main_conveyor import hal__main_conveyor
# from raspi_hal__intake import hal__intake

# from disc_record import *
# from node_templates import *
# from ui_constants import DebuggingButtons, ControlButtons

# from raspi_disc_tracker import raspi_disc_tracker as rdt

import rospy
import cv2
from raspi_hal__turntable import hal__turntable
from raspi_hal__turntable import TURNTABLE_STATE
from enum import Enum
from std_msgs.msg import Int8

class PROCESS_STATE(Enum):
    IDLE = 0,
    MEASURING = 1,
    ADVANCING = 2,

    # MOVING_OUTTAKE_TO_BOX = 2,
    # MOVING_MAIN_CONVEYOR = 3,
    # MOVING_INTAKE_TOP_CONVEYOR = 4,
# MOTION_STATES = [PROCESS_STATE.MOVING_OUTTAKE_TO_BOX, PROCESS_STATE.MOVING_MAIN_CONVEYOR, PROCESS_STATE.MOVING_INTAKE_TOP_CONVEYOR]

class raspi_main:
    def __init__(self):
        rospy.init_node('raspi_main')
        rospy.loginfo("raspi_main node started")
        # self.camera1 = cv2.VideoCapture(0)
        # self.camera1.set(cv2.CAP_PROP_FRAME_WIDTH, 1200)
        # self.camera1.set(cv2.CAP_PROP_FRAME_HEIGHT, 1920)
        # self.camera1.set(cv2.CAP_PROP_FPS, 1)
        # self.camera2 = cv2.VideoCapture(2)
        # self.camera2.set(cv2.CAP_PROP_FRAME_WIDTH, 1200)
        # self.camera2.set(cv2.CAP_PROP_FRAME_HEIGHT, 1920)
        # self.camera2.set(cv2.CAP_PROP_FPS, 1)

        # --- HAL (Hardware Abstraction Layer) ---

        # calls hal__turntable (imported from raspi_hal__turntable) with a callback routine which can be called from hal__turntable
        rospy.loginfo("Creating hal__turntable")
        self.hal__turntable = hal__turntable(self._callback__turntable_complete)

        # --- Subscribers ---
        # self.button_b_subscriber = rospy.Subscriber('ui_button', String, self.ui_callback)

        # --- Publishers ---
        # self.top_conveyor_publisher = rospy.Publisher('top_conveyor_tracker', String, queue_size=10)
        # self.main_conveyor_publisher = rospy.Publisher('main_conveyor_tracker', String, queue_size=10)

        # --- Disc Tracker ---
        # self.rdt = rdt(self.top_conveyor_publisher, self.main_conveyor_publisher)

        # --- Discs --- # TODO: update this to include the new code about the disc tracker. Namely, the get_disc_by_location needs to be updated because it is used below in the callbacks from the nucleos
        # self.discs = []
        # self.new_disc = lambda : disc_record()
        # self.get_disc_by_location = lambda location : next((disc for disc in self.discs if disc.location == location), None)

        # --- State Machine ---
        self.state = PROCESS_STATE.IDLE


    # -- meta state machine -- these functions are part of the state machine for the entire machine
    # def check_state_transition(self):
    #     #TODO: Needs to be fleshed out.

    #     rospy.loginfo("check_state_transition called!!!!!")

    #     if self.state == PROCESS_STATE.IDLE:
    #         pass 
    #         # TODO: Eventually have this function check the status of hals, and notify if any go down? 
    #         # This is never really called, because the event of this function is only started when a module completes, or it is triggered manuall (in self.advance)


    #     # TODO: running into an issue where the state is still in idle by the time the pi gets here in the code. It is about to move into its first state on the nucleo side, but I could force it into that state early on the py side so it doesn't think its still in idle here? 
    #     # TODO: ^^^ ask lewin 
    #     elif self.state == PROCESS_STATE.ADVANCING: 
    #         # check to ensure all advancing modules are back in idle 
    #         if ((self.hal__intake.state.value == 0) and
    #             (self.hal__main_conveyor.state.value == 0) and
    #             (self.hal__outtake.state.value == 0) and
    #             (self.hal__box_conveyor.state.value == 0) 
    #                ):
    #             self.state = PROCESS_STATE.MEASURING  
    #             print("moved to measuring")                   

    #     elif self.state == PROCESS_STATE.MEASURING:
    #         if all([hal.complete() for hal in self.HALs_measure.values()]):
    #             self.state = PROCESS_STATE.MOVING_OUTTAKE_TO_BOX
    #             # TODO: Start outtake process

    #     elif self.state == PROCESS_STATE.MOVING_INTAKE_TOP_CONVEYOR:
    #         if all([hal.complete() for hal in (self.hal__intake, self.hal__main_conveyor)]):
    #             self.state = PROCESS_STATE.MEASURING
    #             self.start_measurement()
        
    #     rospy.loginfo("check_state_transition called. Current meta machine state is: "+str(self.state))    


    # def advance(self): 
    #     # TODO: Ensure this error catching startup code (can_move_discs function) is working  
    #     # if not self.can_move_discs():
    #     #     rospy.logwarn("Cannot move discs, not all nodes are ready")
    #     #     return

    #     # -- now we are ready to move the discs 
    #     self.state = PROCESS_STATE.ADVANCING

    #     #TODO: Eventually start the box conveyor, and that will set off a chain of events through the callbacks box -> outtake -> main -> intake -> main 
    #     # for now, since module C is broken, starting farther up the chain 
    #     self.hal__intake.start()

    #     self.check_state_transition() # this will be called again by the hal callbacks 



        
        # - move the outtake, wait for response 
        # - move the conveyor, wait for response 
        # self.hal__main_conveyor.start()

        # # - move the intake (because we know that)
        # self.hal__intake.start() 
        # rospy.loginfo("INTAKE BUTTON PRESSED HOORAY")
        
        # self.hal_turntable.picture_disc("Test", "testDisc")


    # -- checking functions -- 
    # def can_move_discs(self) -> bool:
    #     return all([motion_hal.ready() for motion_hal in self.HALs_motion.values()]) \
    #         and all([measure_hal.complete() for measure_hal in self.HALs_measure.values()])

    # def can_start_measurement(self) -> bool:
    #     rospy.loginfo("starting something")
    #     return all([measure_hal.ready() for measure_hal in self.HALs_measure.values()]) \
    #         and all([motion_hal.complete() for motion_hal in self.HALs_motion.values()])
    
    # def start_measurement(self):
    #     for measure_hal in self.HALs_measure.values():
    #         measure_hal.start()

    # TODO: probably just move this function into raspi_disc_tracker.py    
    # def queue_complete(self) -> bool:
    #     return False # TODO: Implement checking for when top disc conveyor is empty
    
    

    # -- hal callbacks -- # called when the module completes 
    # def _callback__scale_complete(self, node_name:str):
    #     rospy.loginfo("* SCALE measurement complete, Notified via callback")
    #     if disc := self.get_disc_by_location(location.MAIN_CONVAYOR__SCALE):
    #         disc.weight = self._hal__scale.get_weight()
    #     self.check_state_transition()
    
    # def _callback__flex_complete(self, node_name:str):
    #     rospy.loginfo("* FLEX measurement complete, Notified via callback")
    #     if disc := self.get_disc_by_location(location.MAIN_CONVAYOR__FLEXIBILITY):
    #         disc.flex = self._hal__flex.get_flex()
    #     self.check_state_transition()
    
    # def _callback__height_complete(self, node_name:str):
    #     rospy.loginfo("* HEIGHT measurement complete, Notified via callback")
    #     if disc := self.get_disc_by_location(location.MAIN_CONVAYOR__FLEXIBILITY):
    #         disc.height = self._hal__height.get_height()
    #     self.check_state_transition()
    
    def _callback__turntable_complete(self, node_name:str):
        rospy.loginfo("* TURNTABLE motion complete, Notified via callback")
        self.check_state_transition()

    # def _callback__label_tamper_complete(self, node_name:str):
    #     rospy.loginfo("* LABELER motion complete, Notified via callback")
    #     self.check_state_transition()






    # -- TODO: There has got to be a more readable way to do state transitions than just throwing them all in a callback like this. 
    # -- TODO: Ask Lewin if there is any way we can clean this up, and make it more readable as a meta-state diagram, rather than a bunch of callbacks 
    # def _callback__intake_complete(self, node_name:str):
    #     rospy.loginfo("* INTAKE motion complete, Notified via callback")
    #     self.check_state_transition()

    # def _callback__main_conveyor_complete(self, node_name:str):
    #     rospy.loginfo("* MAIN CONVEYOR motion complete, Notified via callback")
        
    #     if self.state == PROCESS_STATE.ADVANCING: # if we are in the advancing mode, not staying in idle for testing # start up the next module 
    #         pass # done with advancing! 

    #     self.check_state_transition()
    
    # def _callback__outtake_complete(self, node_name:str):
    #     rospy.loginfo("* OUTTAKE motion complete, Notified via callback")

    #     if self.state == PROCESS_STATE.ADVANCING: # if we are in the advancing mode, not staying in idle for testing # start up the next module 
    #         self.hal__main_conveyor.start() 

    #     self.check_state_transition()
        
    # def _callback__box_conveyor_complete(self, node_name:str):
    #     rospy.loginfo("* BOX CONVEYOR motion complete, Notified via callback")
    #     self.check_state_transition()
        
    # this is part of a side interaction between the main conveyor and intake, main -> intake -> main
    # def _callback_main_conveyor_ready_for_intake(self):
    #         rospy.loginfo("* MAIN CONVEYOR ready for intake, Notified via callback")
    #         self.hal__intake.start()
    #         self.state = PROCESS_STATE.MOVING_INTAKE_TOP_CONVEYOR

    # def _callback_intake_ready_for_main_conveyor(self):
    #         rospy.loginfo("* INTAKE ready for main conveyor, Notified via callback")
    #         self.hal__main_conveyor.start()
    #         # No state change since main conveyor is only finishing motion (with centering)
                    
        
    # -- handle incoming messages from UI node -- 
    # def ui_callback(self, btn:String):
    #     """
    #     Handles the incoming messages from the UI node
    #     This method is a callback bound to a Subscriber on the /ui_button topic.
    #     :param btn    [String]    The string assigned to the UI button in ui_constants and raspi_ui
    #     """
    #     rospy.loginfo("[main] UI Button " + btn.data + " Pressed")
        
    #     # Stop all units
    #     if btn.data == ControlButtons.STOP.name:
    #         for hal in self.HALs.values():
    #             hal.request(REQUEST.STOP)   
        
    #     elif btn.data == ControlButtons.ADVANCE.name: 
    #         self.advance()

    #     elif btn.data == ControlButtons.INCREASE_TOP_CONVEYOR.name:
    #         self.rdt.new_disc()
    #     elif btn.data == ControlButtons.DECREASE_TOP_CONVEYOR.name:
    #         self.rdt.remove_last_disc()
    #     elif btn.data == DebuggingButtons.MOVE_TRACKER_FORWARD.name:
    #         self.rdt.move_all_measures_over()

        
    #     elif btn.data == DebuggingButtons.CAMERA10DEGREES.name:
    #         self.hal__turntable.picture_disc(cv2.VideoCapture(4))
    #         rospy.loginfo("should be showing camera :)")

    #     elif btn.data == DebuggingButtons.CAMERA35DEGREES.name:
    #         self.hal__turntable.picture_disc(cv2.VideoCapture(0))
        
    #     elif btn.data == DebuggingButtons.CAMERA90DEGREES.name:
    #         self.hal__turntable.picture_disc(cv2.VideoCapture(2))


    #     elif btn.data == DebuggingButtons.TURNTABLE_START.name:
    #         self.hal__turntable.start() 
        
    #     elif btn.data == DebuggingButtons.CONVEYOR_START.name:
    #         self.hal__main_conveyor.start()

    #     elif btn.data == DebuggingButtons.INTAKE_START.name:
    #         self.hal__intake.start()

    #     elif btn.data == DebuggingButtons.FLEX_START.name:
    #         self.hal__flex.start()


    #     # outtake button 
    #     elif btn.data == DebuggingButtons.BOX_CONVEYOR_START.name:
    #         self.hal__box_conveyor.start()

    #     elif btn.data == DebuggingButtons.OUTTAKE_START.name:
    #         self.hal__outtake.start()

    #     # elif btn.data == UIConstants.MEASURE_START.name:
    #     #     # if not self.can_start_measurement():
    #     #     #     rospy.logwarn("Cannot start measurement, not all nodes are ready")
    #     #     #     return
    #     #     self.start_measurement()
    #     # elif btn.data == UIConstants.MOTION_START.name:
    #     #     # if not self.can_move_discs():
    #     #     #     rospy.logwarn("Cannot move discs, not all nodes are ready")
    #     #     #     return
    #     #     pass
    #     #     # self.move_discs()

    
    # # TODO: is this function actually used anywhere? 
    # def nucleo_response_callback(self, msg):
    #     print("[main] Nucleo Response Received: " + msg.data)

    
    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        #rospy.spin()
        while True:
            self.hal__turntable.get_images()
        #self.hal__turntable.get_images2()
        # rospy.sleep(2)
        # self.hal__turntable.get_images()

if __name__ == '__main__':
    main = raspi_main()
    main.run()
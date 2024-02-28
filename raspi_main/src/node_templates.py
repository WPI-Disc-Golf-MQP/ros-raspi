#!/usr/bin/env python3


from abc import ABC, abstractmethod
from enum import Enum
from threading import Timer
from datetime import datetime, timedelta
from typing import Callable, Type
import logging
import rospy
from std_msgs.msg import Int8

class MODULE_STATUS(Enum):
    INITIALIZING_MODULE = 0
    IDLE = 1
    IN_PROGRESS = 2
    COMPLETE = 3
    INVALID_REPEAT = 4
    ERROR_REBOOT = 5

class REQUEST(Enum):
    INITIALIZING = 0
    STOP = 1
    START = 2
    VERIFY_COMPLETE = 3
    CALIBRATE = 4
    REBOOT = 5

class serial_node(ABC):
    def __init__(self, NAME:str, STATE_TYPE:Type[Enum],
                 COMPLETION_CALLBACK:Callable[[str], None]=lambda _: None,
                 STATE_CHANGE_CALLBACK:Callable[[int, int], None]=lambda _,__: None,
                 STATE_TOPIC:"tuple[str, type]"=("",type(None)),
                 STATUS_TOPIC:"tuple[str, type]"=("",type(None)),
                 REQUEST_TOPIC:"tuple[str, type]"=("",type(None)),
                 timeout:float=2, 
                 refresh_rate:float=0.25):
        self.name = NAME 
        self.state = STATE_TYPE(0)
        self._state_type = STATE_TYPE
        self.timeout = timeout
        self.completion_callback = COMPLETION_CALLBACK
        self.state_change_callback = STATE_CHANGE_CALLBACK
        self.refresh_rate = refresh_rate
        self.offline_callback = None
        self.online_callback = None
        self.last_online = False
        self.status = MODULE_STATUS.INITIALIZING_MODULE
        
        if STATUS_TOPIC == ("", type(None)):
            STATUS_TOPIC = (NAME.lower() + "_status", Int8)
        if REQUEST_TOPIC == ("", type(None)):
            REQUEST_TOPIC = (NAME.lower() + "_request", Int8)
        if STATE_TOPIC == ("", type(None)): 
            STATE_TOPIC = (NAME.lower() + "_state", Int8)
            
        if STATUS_TOPIC[0] != "" and STATUS_TOPIC[1] is not type(None):
            self.watchdog_sub = rospy.Subscriber(*STATUS_TOPIC, self.update)
        if REQUEST_TOPIC[0] != "" and REQUEST_TOPIC[1] is not type(None):
            self.request_pub = rospy.Publisher(*REQUEST_TOPIC, queue_size=10)
        if STATE_TOPIC[0] != "" and STATE_TOPIC[1] is not type(None):
            self.state_sub = rospy.Subscriber(*STATE_TOPIC, self.recieve_state)

        self.last_seen = datetime.now() - timedelta(seconds=5)
        self.last_complete = datetime.now() - timedelta(seconds=5)
        self.is_online = lambda : self.last_seen is not None and \
            (datetime.now() - self.last_seen) < timedelta(seconds=timeout)
        
        self.timer = Timer(refresh_rate, self._check_callbacks)
        self.timer.start()
        
        self.request(REQUEST.INITIALIZING)
    
    def request(self, request:REQUEST):
        self.request_pub.publish(request.value)
        self.last_request = request

    def update(self, msg):
        # rospy.loginfo("[" + self.name + "] Status Update: " + str(msg.data) + ", last seen " + str(self.last_online))
        self.last_seen = datetime.now()
        try:
            new_status = MODULE_STATUS(msg.data)
        except:
            new_status = MODULE_STATUS.IDLE
            rospy.logerr("[" + self.name + "] Invalid status message received: " + str(msg.data))
            
        if new_status != self.status:
            rospy.loginfo("[" + self.name + "] Status Change: " + str(self.status) + " -> " + str(new_status))
            self.status = new_status
            if self.status == MODULE_STATUS.COMPLETE:
                self.last_complete = datetime.now()
                if self.completion_callback is not None: self.completion_callback(self.name)
            
        self._check_callbacks()
    
    def recieve_state(self, msg:Int8):
        # try:
            if self.state_change_callback is not None: self.state_change_callback(self.state, msg.data)
            self.state = self._state_type(msg.data)
        # except Exception as e:
        #    rospy.logerr("[" + self.name + "] Error in state message or callback handling: " + str(msg.data))
        #    logging.exception("[" + self.name + "] Error in state message or callback handling")

    def get_state(self) -> int:
        return int(self.state)
    
    def get_status(self) -> MODULE_STATUS:
        return self.status
    
    def set_offline_callback(self, callback:Callable):
        self.offline_callback = callback

    def set_online_callback(self, callback:Callable):
        self.online_callback = callback

    def _check_callbacks(self):
        # rospy.loginfo("last_online: " + str(self.last_online) + ", is_online: "+ str(self.is_online()) + ", online_callback: " + str(type(self.online_callback)) + ", offline_callback: " + str(type(self.offline_callback)))
        
        if not self.last_online and self.is_online():
            rospy.logwarn("[" + self.name + "] Now online")
            if self.online_callback is not None: self.online_callback()
        elif self.last_online and not self.is_online():
            rospy.logerr("[" + self.name + "] Now offline")
            if self.offline_callback is not None: self.offline_callback()
            
        if self.timer.is_alive: self.timer.cancel()
        self.timer = Timer(self.refresh_rate, self._check_callbacks)
        self.timer.start()
        self.last_online = self.is_online()
    
    
    def start(self):
        self.request(REQUEST.START)
        rospy.loginfo("[" + self.name + "] Motion Started")
        
    def stop(self):
        self.request(REQUEST.STOP)
        
    def complete(self) -> bool:
        return self.verify_complete()
    
    def ready(self) -> bool:
        return self.status == MODULE_STATUS.IDLE
    
    def verify_complete(self) -> bool:
        if (self.status != MODULE_STATUS.IN_PROGRESS and
            (datetime.now() - self.last_complete) > timedelta(seconds=self.timeout)):
            self.request(REQUEST.VERIFY_COMPLETE)
            return False
        return datetime.now() - self.last_complete < timedelta(seconds=self.timeout)

class measure_node(serial_node): 
    pass

class motion_node(serial_node):
    pass
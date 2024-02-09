#!/usr/bin/env python3


from abc import ABC, abstractmethod
from enum import Enum
from threading import Timer
from datetime import datetime, timedelta
from typing import Callable
import rospy
from std_msgs.msg import Int8

class NODE_STATUS(Enum):
    INITIALIZING_NODE = 0
    IDLE = 1
    MOTION_IN_PROGRESS = 2
    MOTION_COMPLETE = 3
    MEASURE_IN_PROGRESS = 4
    MEASURE_COMPLETE = 5
    INVALID_MEASURE__REPEAT = 6
    ERROR__REBOOT = 7

class REQUEST(Enum):
    INITIALIZING = 0
    WAITING = 1
    START_MOTION = 2
    VERIFY__MOTION_COMPLETE = 3
    START_MEASURE = 4
    VERIFY__MEASURE_COMPLETE = 5
    REBOOT = 6

class serial_node(ABC):
    def __init__(self, NAME:str, 
                 COMPLETION_CALLBACK:Callable[[str], None]=lambda _: None,
                 STATUS_TOPIC:"tuple[str, type]"=("",type(None)),
                 REQUEST_TOPIC:"tuple[str, type]"=("",type(None)),
                 timeout:float=2, 
                 refresh_rate:float=0.25):
        self.name = NAME 
        self.timeout = timeout
        self.completion_callback = COMPLETION_CALLBACK
        self.refresh_rate = refresh_rate
        self.offline_callback = None
        self.online_callback = None
        self.last_online = False
        self.status = NODE_STATUS.INITIALIZING_NODE
        
        if STATUS_TOPIC == ("", type(None)):
            STATUS_TOPIC = (NAME.lower() + "_status", Int8)
        if REQUEST_TOPIC == ("", type(None)):
            REQUEST_TOPIC = (NAME.lower() + "_request", Int8)
            
        if STATUS_TOPIC[0] != "" and STATUS_TOPIC[1] is not type(None):
            self.watchdog_sub = rospy.Subscriber(*STATUS_TOPIC, self.update)
        if REQUEST_TOPIC[0] != "" and REQUEST_TOPIC[1] is not type(None):
            self.request_pub = rospy.Publisher(*REQUEST_TOPIC, queue_size=10)

        self.last_seen = datetime.now() - timedelta(seconds=5)
        self.last_motion_complete = datetime.now() - timedelta(seconds=5)
        self.last_measure_complete = datetime.now() - timedelta(seconds=5)
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
            new_status = NODE_STATUS(msg.data)
        except:
            new_status = NODE_STATUS.IDLE
            rospy.logerr("[" + self.name + "] Invalid status message received: " + str(msg.data))
            
        if new_status != self.status:
            rospy.loginfo("[" + self.name + "] Status Change: " + str(self.status) + " -> " + str(new_status))
            self.status = new_status
            if self.status == NODE_STATUS.MOTION_COMPLETE:
                self.last_motion_complete = datetime.now()
                if self.completion_callback is not None: self.completion_callback(self.name)
            elif self.status == NODE_STATUS.MEASURE_COMPLETE:
                self.last_measure_complete = datetime.now()
                if self.completion_callback is not None: self.completion_callback(self.name)
            
        self._check_callbacks()

    def get_status(self) -> NODE_STATUS:
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
    

class measure_node(serial_node):
    @abstractmethod
    def get(self):
        pass
    
    @abstractmethod
    def valid_measurement(self) -> bool:
        pass
    
    def start(self):
        self.request(REQUEST.START_MEASURE)
        rospy.loginfo("[" + self.name + "] Measure Started")
    
    def complete(self) -> bool:
        return self.verify_measure_complete()
    
    def ready(self) -> bool:
        return self.status == NODE_STATUS.IDLE
    
    def verify_measure_complete(self) -> bool:
        if (self.status != NODE_STATUS.MEASURE_IN_PROGRESS and 
            (datetime.now() - self.last_measure_complete) > timedelta(seconds=self.timeout)):
            self.request(REQUEST.VERIFY__MEASURE_COMPLETE)
            return False
        return self.status == NODE_STATUS.MEASURE_COMPLETE and self.valid_measurement()

class motion_node(serial_node):
    def stop(self):
        self.request(REQUEST.WAITING)
        
    def complete(self) -> bool:
        return self.verify_motion_complete()
    
    def ready(self) -> bool:
        return self.status == NODE_STATUS.IDLE

    def start(self):
        self.request(REQUEST.START_MOTION)
        rospy.loginfo("[" + self.name + "] Motion Started")
    
    def verify_motion_complete(self) -> bool:
        if (self.status != NODE_STATUS.MOTION_IN_PROGRESS and
            (datetime.now() - self.last_motion_complete) > timedelta(seconds=self.timeout)):
            self.request(REQUEST.VERIFY__MOTION_COMPLETE)
            return False
        return datetime.now() - self.last_motion_complete < timedelta(seconds=self.timeout)
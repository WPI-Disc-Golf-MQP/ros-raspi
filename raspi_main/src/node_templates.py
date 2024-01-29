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


class measure_node(ABC):
    @abstractmethod
    def get(self):
        pass
    
    @abstractmethod
    def can_start_measurement(self):
        pass
    
    @abstractmethod
    def start(self):
        pass
    
    @abstractmethod
    def complete(self) -> bool:
        pass

class motion_node(ABC):
    def __init__(self):
        self.enabled = False
        pass

    def enable(self):
        self.enabled = True

    def disable(self):
        self.enabled = False

    def is_enabled(self) -> bool:
        return self.enabled

    @abstractmethod
    def stop(self):
        pass
    
    @abstractmethod
    def can_start_motion(self):
        pass

    @abstractmethod
    def start(self):
        pass
    
    @abstractmethod
    def complete(self) -> bool:
        pass

class serial_node(ABC):
    def __init__(self, NAME:str, 
                 STATUS_TOPIC:"tuple[str, type]"=("",type(None)),
                 timeout:float=2, 
                 refresh_rate:float=0.25):
        self.name = NAME 
        self.refresh_rate = refresh_rate
        self.offline_callback = None
        self.online_callback = None
        self.last_online = False
        self.status = NODE_STATUS.INITIALIZING
        
        if STATUS_TOPIC == ("", type(None)):
            STATUS_TOPIC = (NAME.lower() + "_status", Int8)
        if STATUS_TOPIC[0] != "" and STATUS_TOPIC[1] is not type(None):
            self.watchdog_sub = rospy.Subscriber(*STATUS_TOPIC, self.update)

        self.last_seen = datetime.now() - timedelta(seconds=5)
        self.is_online = lambda : self.last_seen is not None and \
            (datetime.now() - self.last_seen) < timedelta(seconds=timeout)
        
        self.timer = Timer(refresh_rate, self._check_callbacks)
        self.timer.start()

    def update(self, msg):
        # rospy.loginfo("[" + self.name + "] Status Update: " + str(msg.data) + ", last seen " + str(self.last_online))
        self.last_seen = datetime.now()
        try:
            self.status = NODE_STATUS(msg.data)
        except:
            self.status = NODE_STATUS.ERROR
            rospy.logerr("[" + self.name + "] Invalid status message received: " + str(msg.data))
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
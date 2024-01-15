#!/usr/bin/env python3


from abc import ABC, abstractmethod
from enum import Enum
from threading import Timer
import time
from typing import Callable
from typing_extensions import final

import rospy
from std_msgs.msg import Int8

class NODE_STATUS(Enum):
    INITIALIZING = 0
    ONLINE_IDLE = 1
    REQUEST_IN_PROGRESS = 2
    REQUEST_COMPLETE = 3
    INVALID__REPEAT_REQUIRED = 4
    ERROR = 5

class REQUEST(Enum):
    INITIALIZING = 0
    ONLINE_IDLE = 1
    REQUEST__STANDARD = 2
    RESET__STANDARD = 3
    REQUEST__REPEAT = 4
    REBOOT = 5


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
    def __init__(self, NAME:str, STATUS_TOPIC:tuple[str, type]=("",type(None)),
                  timeout:float=2, 
                  refresh_rate:float=0.25):
        self.name = NAME
        if STATUS_TOPIC == ("", type(None)):
            STATUS_TOPIC = (NAME.lower() + "_status", Int8)
        self.last_time:float = 0
        self.is_online = lambda : self.last_online is not None and (time.time() - self.last_online) < timeout
        self.last_online = False
        self.status = NODE_STATUS.INITIALIZING
        if STATUS_TOPIC[0] is str and STATUS_TOPIC[0] != "" and STATUS_TOPIC[1] is not type(None):
            self.watchdog_sub = rospy.Subscriber(*STATUS_TOPIC, self.update)
        self.refresh_rate = refresh_rate
        self.timer = Timer(refresh_rate, self._check_callbacks)
        self.timer.start()

    def set_offline_callback(self, callback:Callable):
        self.offline_callback = callback

    def set_online_callback(self, callback:Callable):
        self.online_callback = callback

    def update(self, msg):
        self.last_time = time.time()
        try:
            self.status = NODE_STATUS[msg.data]
        except:
            self.status = NODE_STATUS.ERROR
            rospy.logerr("[" + self.name + "] Invalid status message received: " + msg.data)
        self._check_callbacks()

    def get_status(self) -> NODE_STATUS:
        return self.status

    def _check_callbacks(self):
        if not self.last_online and self.is_online() and self.online_callback is not None:
            self.online_callback()
        elif self.last_online and not self.is_online() and self.offline_callback is not None:
            self.offline_callback()

        self.last_online = self.is_online()

        if self.timer.is_alive: self.timer.cancel()
        self.timer = Timer(self.refresh_rate, self._check_callbacks)
        self.timer.start()
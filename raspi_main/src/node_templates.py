#!/usr/bin/env python3


from abc import ABC, abstractmethod
from threading import Timer
import time
from typing import Callable, Type
from typing_extensions import final

import rospy


class measure_node(ABC):
    def __init__(self):
        self.enabled = False
        pass

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
    def __init__(self, watchdog_sub_topic:str="", watchdog_sub_type:Type=Type, timeout:float=2, refresh_rate:float=0.25):
        self.last_time:float = 0
        self.online = lambda : self.last_online is not None and (time.time() - self.last_online) < timeout
        self.last_online = False
        if watchdog_sub_topic is str and watchdog_sub_topic != "":
            self.watchdog_sub = rospy.Subscriber(watchdog_sub_topic, watchdog_sub_type, self.update)
        self.refresh_rate = refresh_rate
        self.timer = Timer(refresh_rate, self._check_callbacks)
        self.timer.start()

    def set_offline_callback(self, callback:Callable):
        self.offline_callback = callback

    def set_online_callback(self, callback:Callable):
        self.online_callback = callback

    def update(self, msg):
        self.last_time = time.time()
        self._check_callbacks()

    def _check_callbacks(self):
        if not self.last_online and self.online() and self.online_callback is not None:
            self.online_callback()
        elif self.last_online and not self.online() and self.offline_callback is not None:
            self.offline_callback()

        self.last_online = self.online()

        if self.timer.is_alive: self.timer.cancel()
        self.timer = Timer(self.refresh_rate, self._check_callbacks)
        self.timer.start()
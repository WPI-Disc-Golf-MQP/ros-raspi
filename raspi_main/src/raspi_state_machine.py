#!/usr/bin/env python3

from sre_parse import State
from tracemalloc import start
from typing import Callable
from statemachine import StateMachine, State

class raspi_state_machine(StateMachine):
    init = State(initial=True)
    moving = State()
    measuring = State()
    idle = State()

    start_motion = moving.to(moving, cond="can_start_motion")| \
            init.to(moving, cond="can_start_motion")| \
            idle.to(moving, cond="can_start_motion")
    start_measurement = moving.to(measuring, cond="can_start_measurement")| \
        init.to(measuring, cond="can_start_measurement")| \
        idle.to(measuring, cond="can_start_measurement")
    idle = moving.to(idle, cond="can_idle")| \
        measuring.to(idle, cond="can_idle")
    
    cycle = start_motion | start_measurement

    def __init__(self, can_move_discs:Callable, move_discs:Callable, can_start_measurement:Callable, start_measurement:Callable, can_idle:Callable):
        self.can_start_motion = self.can_start_motion
        self.on_enter_moving = self.start_motion
        self.can_start_measurement = self.can_start_measurement
        self.on_enter_measuring = self.start_measurement
        self.can_idle = self.can_idle
        super().__init__()
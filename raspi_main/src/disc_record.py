#!/usr/bin/env python3


from dataclasses import dataclass, field
from enum import Enum

class location(Enum):
    TOP_CONVEYOR = 0
    INTAKE__RAMP = 1
    INTAKE__BOTTOM = 2
    MAIN_CONVAYOR__START = 3
    MAIN_CONVAYOR__TURNTABLE = 4
    MAIN_CONVAYOR__CV = 5
    MAIN_CONVAYOR__FLEXIBILITY = 6
    MAIN_CONVAYOR__SCALE = 6
    OUTTAKE__TOP = 7
    OUTTAKE__LABELER = 8
    OUTTAKE__BOTTOM = 9
    EXIT = 10

@dataclass
class disc_record:
    diameter: float = -1
    height: float = -1
    flexibility: float = -1
    weight: float = -1

    sku: str = "(None)"
    index: int = -1
    handle: str = "(None)"

    loc: location = location.TOP_CONVEYOR

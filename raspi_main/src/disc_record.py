#!/usr/bin/env python3


from dataclasses import dataclass, field
from enum import Enum

class location(Enum):
    TOP_CONVEYOR = 0
    INTAKE = 1
    MAIN_CONVAYOR__TURNTABLE = 2
    MAIN_CONVAYOR__CV = 3
    MAIN_CONVAYOR__FLEXIBILITY = 4
    MAIN_CONVAYOR__SCALE = 5
    OUTTAKE = 6
    BOXES = 7

@dataclass
class disc_record:
    sku: str = "(No SKU)" # unique for each manufacturer classification
    index: int = -1 # unique for each disc 

    def handle(self):
        # handle: str = "(None)" # unique for each disc - sku + index
        return self.sku +"-"+ str(self.index)

    diameter: float = -1
    height: float = -1
    flexibility: float = -1
    weight: float = -1

    loc: location = location.TOP_CONVEYOR
    top_conveyor__sub_loc: float = -1
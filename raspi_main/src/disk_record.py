#!/usr/bin/env python3


@dataclass
def disk_record:
    measurement: measurement_data = measurement_data()
    user_data: user_data = user_data()
    location: location = location.TOP_CONVEYOR

@dataclass
class measurement_data:
    diameter: float
    height: float
    flexibility: float
    weight: float

@dataclass
class user_data:
    sku: str
    index: int
    handle: str


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
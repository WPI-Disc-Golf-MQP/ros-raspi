from enum import Enum


class DebuggingButtons(Enum):
    CAMERA10DEGREES = "Camera10Degrees",
    CAMERA35DEGREES = "Camera35Degrees",
    CAMERA90DEGREES = "Camera90Degrees",

    TURNTABLE_START = "Start Turntable",
    CONVEYOR_START = "Start Conveyor",
    INTAKE_START = "Start Intake",
    FLEX_START = "Start Flex",

    MOVE_TRACKER_FORWARD = "MOVE_STORAGE_FORWARD",

    TRISTANS_BUTTON = "run outtake",


class ControlButtons(Enum):
    # SEQUENCE_START = "Start Sequence",
    # HOME_ALL = "Home All",

    STOP = "Stop All",
    ADVANCE = "Advance", 

    INCREASE_TOP_CONVEYOR = "INCREASE_TOP_CONVEYOR",
    DECREASE_TOP_CONVEYOR = "DECREASE_TOP_CONVEYOR",

    # MEASURE_START = "Start All Measurements",
    # MOTION_START = "Start All Disc Motions",

from enum import Enum

class INTAKE_STATE(Enum):
    INTAKE_IDLE = 0
    INTAKE_SENDING = 1
    INTAKE_RECIEVING = 2

a = INTAKE_STATE.INTAKE_IDLE


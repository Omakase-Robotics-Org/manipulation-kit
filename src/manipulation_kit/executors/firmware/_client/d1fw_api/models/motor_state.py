from enum import Enum


class MotorState(str, Enum):
    ENGAGED = "engaged"
    RELEASED = "released"
    UNKNOWN = "unknown"

    def __str__(self) -> str:
        return str(self.value)

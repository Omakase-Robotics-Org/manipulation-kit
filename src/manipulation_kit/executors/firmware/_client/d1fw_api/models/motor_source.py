from enum import Enum


class MotorSource(str, Enum):
    THIS_PROCESS = "this_process"
    UNKNOWN = "unknown"

    def __str__(self) -> str:
        return str(self.value)

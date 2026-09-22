from enum import Enum


class EyeCmdRequestEffect(str, Enum):
    BREATHING = "breathing"
    CLEAR = "clear"
    RUNNING = "running"
    SOLID = "solid"
    STOP = "stop"

    def __str__(self) -> str:
        return str(self.value)

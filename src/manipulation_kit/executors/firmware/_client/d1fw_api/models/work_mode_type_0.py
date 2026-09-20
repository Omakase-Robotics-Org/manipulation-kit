from enum import Enum


class WorkModeType0(str, Enum):
    IDLE = "idle"

    def __str__(self) -> str:
        return str(self.value)

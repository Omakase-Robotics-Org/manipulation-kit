from enum import Enum


class ArmModeType0(str, Enum):
    IDLE = "idle"

    def __str__(self) -> str:
        return str(self.value)

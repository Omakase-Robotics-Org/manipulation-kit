from enum import Enum


class ArmModeRequestType0Mode(str, Enum):
    IDLE = "idle"

    def __str__(self) -> str:
        return str(self.value)

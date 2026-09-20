from enum import Enum


class ArmModeRequestType2Mode(str, Enum):
    TORQUE = "torque"

    def __str__(self) -> str:
        return str(self.value)

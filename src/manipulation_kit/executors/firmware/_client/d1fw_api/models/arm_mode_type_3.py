from enum import Enum


class ArmModeType3(str, Enum):
    TORQUE = "torque"

    def __str__(self) -> str:
        return str(self.value)

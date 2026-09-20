from enum import Enum


class ArmModeType1(str, Enum):
    POSITION = "position"

    def __str__(self) -> str:
        return str(self.value)

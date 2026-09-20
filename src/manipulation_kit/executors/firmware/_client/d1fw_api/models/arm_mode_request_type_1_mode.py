from enum import Enum


class ArmModeRequestType1Mode(str, Enum):
    POSITION = "position"

    def __str__(self) -> str:
        return str(self.value)

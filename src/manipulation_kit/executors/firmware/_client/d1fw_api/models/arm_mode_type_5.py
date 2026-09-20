from enum import Enum


class ArmModeType5(str, Enum):
    ERROR = "error"

    def __str__(self) -> str:
        return str(self.value)

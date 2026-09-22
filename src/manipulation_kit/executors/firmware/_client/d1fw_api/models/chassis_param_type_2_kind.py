from enum import Enum


class ChassisParamType2Kind(str, Enum):
    SLOPE = "slope"

    def __str__(self) -> str:
        return str(self.value)

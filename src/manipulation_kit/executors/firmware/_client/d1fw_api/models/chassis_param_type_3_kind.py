from enum import Enum


class ChassisParamType3Kind(str, Enum):
    NARROW = "narrow"

    def __str__(self) -> str:
        return str(self.value)

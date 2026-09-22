from enum import Enum


class ChassisSpeedProfile(str, Enum):
    LOWORSTRONG = "lowOrStrong"
    MAX = "max"
    NARROW = "narrow"
    SLOPE = "slope"

    def __str__(self) -> str:
        return str(self.value)

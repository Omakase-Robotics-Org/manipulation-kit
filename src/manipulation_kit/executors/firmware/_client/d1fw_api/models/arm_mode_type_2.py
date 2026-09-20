from enum import Enum


class ArmModeType2(str, Enum):
    PVT = "pvt"

    def __str__(self) -> str:
        return str(self.value)

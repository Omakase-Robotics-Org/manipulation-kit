from enum import Enum


class ChassisParamType4Kind(str, Enum):
    DIST_STOP = "dist_stop"

    def __str__(self) -> str:
        return str(self.value)

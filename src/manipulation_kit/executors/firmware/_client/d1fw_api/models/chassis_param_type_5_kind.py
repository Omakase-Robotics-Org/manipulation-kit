from enum import Enum


class ChassisParamType5Kind(str, Enum):
    FOOTPRINT = "footprint"

    def __str__(self) -> str:
        return str(self.value)

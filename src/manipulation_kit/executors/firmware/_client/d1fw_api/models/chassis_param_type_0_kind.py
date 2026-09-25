from enum import Enum


class ChassisParamType0Kind(str, Enum):
    MAX = "max"

    def __str__(self) -> str:
        return str(self.value)

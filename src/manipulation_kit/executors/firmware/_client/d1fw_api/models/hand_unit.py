from enum import Enum


class HandUnit(str, Enum):
    FRAC = "frac"
    WIRE = "wire"

    def __str__(self) -> str:
        return str(self.value)

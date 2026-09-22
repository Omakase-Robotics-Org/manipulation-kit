from enum import Enum


class ChassisCommonFileReadStatus(str, Enum):
    PRESENT = "present"
    UNKNOWN = "unknown"

    def __str__(self) -> str:
        return str(self.value)

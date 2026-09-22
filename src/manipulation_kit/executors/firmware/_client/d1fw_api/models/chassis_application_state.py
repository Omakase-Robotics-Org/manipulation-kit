from enum import Enum


class ChassisApplicationState(str, Enum):
    UNKNOWN = "unknown"

    def __str__(self) -> str:
        return str(self.value)

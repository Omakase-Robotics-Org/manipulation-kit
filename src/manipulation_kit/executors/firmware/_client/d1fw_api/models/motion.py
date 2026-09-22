from enum import Enum


class Motion(str, Enum):
    ACTIVE = "active"
    IDLE = "idle"
    UNKNOWN = "unknown"

    def __str__(self) -> str:
        return str(self.value)

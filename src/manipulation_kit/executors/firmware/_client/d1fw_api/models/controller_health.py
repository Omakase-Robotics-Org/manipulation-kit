from enum import Enum


class ControllerHealth(str, Enum):
    DEGRADED = "degraded"
    FAULTED = "faulted"
    OK = "ok"
    UNKNOWN = "unknown"

    def __str__(self) -> str:
        return str(self.value)

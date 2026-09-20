from enum import Enum


class SoftKillOutcome(str, Enum):
    FAILED = "failed"
    OK = "ok"
    TIMEOUT = "timeout"
    UNAVAILABLE = "unavailable"

    def __str__(self) -> str:
        return str(self.value)

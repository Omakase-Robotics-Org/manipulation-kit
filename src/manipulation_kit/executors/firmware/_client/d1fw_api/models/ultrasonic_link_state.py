from enum import Enum


class UltrasonicLinkState(str, Enum):
    CYCLE = "cycle"
    HELD = "held"
    LOST = "lost"

    def __str__(self) -> str:
        return str(self.value)

from enum import Enum


class StrokeKind(str, Enum):
    BLIND = "blind"
    CONTACT = "contact"
    EMPTY = "empty"
    FAULT = "fault"
    GRASP = "grasp"
    LOST = "lost"
    OPEN = "open"
    OVERLOAD = "overload"
    TIMEOUT = "timeout"

    def __str__(self) -> str:
        return str(self.value)

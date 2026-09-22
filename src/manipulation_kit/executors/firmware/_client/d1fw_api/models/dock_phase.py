from enum import Enum


class DockPhase(str, Enum):
    ABNORMAL = "abnormal"
    DOCKED = "docked"
    DOCKING = "docking"
    LEAVING = "leaving"
    NOT_DOCKING = "not_docking"
    UNKNOWN = "unknown"

    def __str__(self) -> str:
        return str(self.value)

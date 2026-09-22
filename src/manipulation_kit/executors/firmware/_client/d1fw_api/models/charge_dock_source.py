from enum import Enum


class ChargeDockSource(str, Enum):
    THIS_PROCESS = "this_process"
    UNKNOWN = "unknown"
    VENDOR = "vendor"

    def __str__(self) -> str:
        return str(self.value)

from enum import Enum


class WsSoftKillEventName(str, Enum):
    SOFT_KILL = "soft_kill"

    def __str__(self) -> str:
        return str(self.value)

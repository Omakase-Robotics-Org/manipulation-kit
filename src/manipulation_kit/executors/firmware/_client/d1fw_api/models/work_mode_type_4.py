from enum import Enum


class WorkModeType4(str, Enum):
    REMOTE_CONTROL = "remote_control"

    def __str__(self) -> str:
        return str(self.value)

from enum import Enum


class WorkModeType1(str, Enum):
    NAVIGATION = "navigation"

    def __str__(self) -> str:
        return str(self.value)

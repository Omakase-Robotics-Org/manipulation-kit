from enum import Enum


class WorkModeType3(str, Enum):
    MAPPING = "mapping"

    def __str__(self) -> str:
        return str(self.value)

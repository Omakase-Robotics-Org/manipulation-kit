from enum import Enum


class AdvisorySeverity(str, Enum):
    INFO = "info"

    def __str__(self) -> str:
        return str(self.value)

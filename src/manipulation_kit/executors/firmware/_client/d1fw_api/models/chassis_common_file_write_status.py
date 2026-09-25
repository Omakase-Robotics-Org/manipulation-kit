from enum import Enum


class ChassisCommonFileWriteStatus(str, Enum):
    OUTCOME_UNKNOWN = "outcome_unknown"
    VERIFIED = "verified"

    def __str__(self) -> str:
        return str(self.value)

from enum import Enum


class ChassisPlansWriteStatus(str, Enum):
    OUTCOME_UNKNOWN = "outcome_unknown"
    VERIFIED = "verified"

    def __str__(self) -> str:
        return str(self.value)

from enum import Enum


class ArmModeRequestType4Mode(str, Enum):
    FORCE_COMPLIANCE = "force_compliance"

    def __str__(self) -> str:
        return str(self.value)

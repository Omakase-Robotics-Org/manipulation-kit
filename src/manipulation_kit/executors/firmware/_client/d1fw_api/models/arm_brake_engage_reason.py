from enum import Enum


class ArmBrakeEngageReason(str, Enum):
    EMERGENCY_STOP = "emergency_stop"
    OPERATOR = "operator"
    RELEASE_ABORTED = "release_aborted"
    SOFT_KILL = "soft_kill"
    WINDOW_ELAPSED = "window_elapsed"

    def __str__(self) -> str:
        return str(self.value)

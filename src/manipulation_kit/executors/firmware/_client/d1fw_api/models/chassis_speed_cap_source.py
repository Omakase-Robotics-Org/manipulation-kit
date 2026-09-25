from enum import Enum


class ChassisSpeedCapSource(str, Enum):
    DAEMON_POLICY = "daemon_policy"
    VENDOR_DECLARED = "vendor_declared"
    VENDOR_UNREADABLE = "vendor_unreadable"

    def __str__(self) -> str:
        return str(self.value)

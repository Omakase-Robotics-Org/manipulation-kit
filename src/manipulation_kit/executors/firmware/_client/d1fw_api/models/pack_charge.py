from enum import Enum


class PackCharge(str, Enum):
    CHARGING = "charging"
    DISCHARGING = "discharging"
    UNDOCUMENTED = "undocumented"
    UNREPORTED = "unreported"

    def __str__(self) -> str:
        return str(self.value)

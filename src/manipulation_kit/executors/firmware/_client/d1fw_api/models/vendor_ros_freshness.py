from enum import Enum


class VendorRosFreshness(str, Enum):
    FRESH = "fresh"
    NEVER_RECEIVED = "never_received"
    STALE = "stale"

    def __str__(self) -> str:
        return str(self.value)

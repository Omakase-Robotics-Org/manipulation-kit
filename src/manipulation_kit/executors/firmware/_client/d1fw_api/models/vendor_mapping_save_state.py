from enum import Enum


class VendorMappingSaveState(str, Enum):
    UNVERIFIED = "unverified"

    def __str__(self) -> str:
        return str(self.value)

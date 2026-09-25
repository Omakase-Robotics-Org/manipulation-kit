from enum import Enum


class VendorRosCommandType3Command(str, Enum):
    NAVIGATION = "navigation"

    def __str__(self) -> str:
        return str(self.value)

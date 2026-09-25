from enum import Enum


class VendorRosCommandType6Command(str, Enum):
    PAUSE = "pause"

    def __str__(self) -> str:
        return str(self.value)

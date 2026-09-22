from enum import Enum


class VendorRosCommandType2Command(str, Enum):
    AUTO_CHARGE = "auto_charge"

    def __str__(self) -> str:
        return str(self.value)

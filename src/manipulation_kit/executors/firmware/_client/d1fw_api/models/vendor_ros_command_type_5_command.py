from enum import Enum


class VendorRosCommandType5Command(str, Enum):
    CHARGE_CONTROL = "charge_control"

    def __str__(self) -> str:
        return str(self.value)

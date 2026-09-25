from enum import Enum


class VendorRosCommandType4Command(str, Enum):
    SLEEP = "sleep"

    def __str__(self) -> str:
        return str(self.value)

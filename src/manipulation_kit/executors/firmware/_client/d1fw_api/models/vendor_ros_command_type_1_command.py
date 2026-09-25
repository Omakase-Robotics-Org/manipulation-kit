from enum import Enum


class VendorRosCommandType1Command(str, Enum):
    MAPPING = "mapping"

    def __str__(self) -> str:
        return str(self.value)

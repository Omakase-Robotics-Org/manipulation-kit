from enum import Enum


class VendorMappingAction(str, Enum):
    CONTINUE = "continue"
    END = "end"
    START = "start"

    def __str__(self) -> str:
        return str(self.value)

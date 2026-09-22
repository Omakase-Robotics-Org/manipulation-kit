from enum import Enum


class ChargeBasis(str, Enum):
    HTTP_CHARGING_STATUS = "http_charging_status"
    NONE = "none"
    PACK = "pack"

    def __str__(self) -> str:
        return str(self.value)

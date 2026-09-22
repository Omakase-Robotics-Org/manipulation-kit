from enum import Enum


class ChargeDockReadStatus(str, Enum):
    INVALID_RESPONSE = "invalid_response"
    TRANSPORT_ERROR = "transport_error"
    UNKNOWN = "unknown"
    VENDOR_READ = "vendor_read"

    def __str__(self) -> str:
        return str(self.value)

from enum import Enum


class VendorRosConnection(str, Enum):
    CONNECTED = "connected"
    CONNECTING = "connecting"
    DISCONNECTED = "disconnected"
    NOT_CONFIGURED = "not_configured"

    def __str__(self) -> str:
        return str(self.value)

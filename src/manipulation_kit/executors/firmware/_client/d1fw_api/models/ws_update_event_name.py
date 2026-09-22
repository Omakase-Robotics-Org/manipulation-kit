from enum import Enum


class WsUpdateEventName(str, Enum):
    UPDATE = "update"

    def __str__(self) -> str:
        return str(self.value)

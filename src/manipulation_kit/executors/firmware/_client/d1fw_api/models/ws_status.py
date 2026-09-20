from enum import Enum


class WsStatus(str, Enum):
    ERROR = "error"
    OK = "ok"

    def __str__(self) -> str:
        return str(self.value)

from enum import Enum


class WsNavEventName(str, Enum):
    NAV = "nav"

    def __str__(self) -> str:
        return str(self.value)

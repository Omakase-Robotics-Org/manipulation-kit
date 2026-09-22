from enum import Enum


class ZeroReference(str, Enum):
    COMMISSIONED = "commissioned"
    UNKNOWN = "unknown"

    def __str__(self) -> str:
        return str(self.value)

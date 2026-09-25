from enum import Enum


class ArmToolSource(str, Enum):
    CLIENT = "client"
    CONFIG = "config"
    NONE = "none"

    def __str__(self) -> str:
        return str(self.value)

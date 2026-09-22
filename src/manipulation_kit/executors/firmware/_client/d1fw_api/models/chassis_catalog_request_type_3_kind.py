from enum import Enum


class ChassisCatalogRequestType3Kind(str, Enum):
    MONITOR = "monitor"

    def __str__(self) -> str:
        return str(self.value)

from enum import Enum


class ChassisCatalogRequestType2Kind(str, Enum):
    PLAN = "plan"

    def __str__(self) -> str:
        return str(self.value)

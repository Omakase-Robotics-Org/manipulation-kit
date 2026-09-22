from enum import Enum


class ChassisCatalogRequestType0Kind(str, Enum):
    TASKS = "tasks"

    def __str__(self) -> str:
        return str(self.value)

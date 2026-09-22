from enum import Enum


class ChassisCatalogRequestType1Kind(str, Enum):
    TIMING_TASKS = "timing_tasks"

    def __str__(self) -> str:
        return str(self.value)

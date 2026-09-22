from enum import Enum


class ChassisCatalogRequestType6Kind(str, Enum):
    CHARGING_STAT_TODAY = "charging_stat_today"

    def __str__(self) -> str:
        return str(self.value)

from enum import Enum


class WorkModeType2(str, Enum):
    AUTO_CHARGING = "auto_charging"

    def __str__(self) -> str:
        return str(self.value)

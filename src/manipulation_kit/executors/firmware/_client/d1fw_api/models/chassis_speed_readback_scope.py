from enum import Enum


class ChassisSpeedReadbackScope(str, Enum):
    AGGREGATE_SAVED_PROFILE = "aggregate_saved_profile"

    def __str__(self) -> str:
        return str(self.value)

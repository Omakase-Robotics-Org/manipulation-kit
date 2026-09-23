from enum import Enum


class TrajectoryGuard(str, Enum):
    FULL = "full"
    SPEED_ONLY = "speed_only"

    def __str__(self) -> str:
        return str(self.value)

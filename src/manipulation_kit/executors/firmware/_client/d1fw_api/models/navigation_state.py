from enum import Enum


class NavigationState(str, Enum):
    BLOCKED = "blocked"
    GOAL_OCCUPIED = "goal_occupied"
    LOCALIZATION_LOST = "localization_lost"
    LOCALIZING = "localizing"
    MOVING = "moving"
    OFF = "off"
    PAUSED = "paused"
    READY = "ready"
    SENSOR_LOST = "sensor_lost"
    UNKNOWN = "unknown"

    def __str__(self) -> str:
        return str(self.value)

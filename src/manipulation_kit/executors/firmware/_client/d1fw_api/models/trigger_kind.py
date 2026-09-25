from enum import Enum


class TriggerKind(str, Enum):
    CHASSIS_ESTOP = "chassis_estop"
    MANUAL = "manual"

    def __str__(self) -> str:
        return str(self.value)

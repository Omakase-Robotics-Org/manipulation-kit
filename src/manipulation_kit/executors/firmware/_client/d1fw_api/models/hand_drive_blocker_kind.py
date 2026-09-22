from enum import Enum


class HandDriveBlockerKind(str, Enum):
    EMERGENCY_SWITCH = "emergency_switch"
    MOTORS_RELEASED = "motors_released"
    NOT_IN_REMOTE_CONTROL = "not_in_remote_control"
    STANDBY = "standby"

    def __str__(self) -> str:
        return str(self.value)

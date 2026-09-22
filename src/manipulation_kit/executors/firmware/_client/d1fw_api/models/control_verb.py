from enum import Enum


class ControlVerb(str, Enum):
    CHARGE = "charge"
    CLOSE_NAVIGATION = "close_navigation"
    ENGAGE_MOTORS = "engage_motors"
    ENTER_REMOTE_CONTROL = "enter_remote_control"
    LEAVE_REMOTE_CONTROL = "leave_remote_control"
    RELEASE_MOTORS = "release_motors"
    START_NAVIGATION = "start_navigation"

    def __str__(self) -> str:
        return str(self.value)

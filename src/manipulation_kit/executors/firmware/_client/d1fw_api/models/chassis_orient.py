from enum import Enum


class ChassisOrient(str, Enum):
    BACK = "back"
    FORWARD = "forward"
    LEFT = "left"
    RIGHT = "right"

    def __str__(self) -> str:
        return str(self.value)

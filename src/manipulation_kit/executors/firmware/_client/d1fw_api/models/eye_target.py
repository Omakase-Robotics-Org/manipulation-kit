from enum import Enum


class EyeTarget(str, Enum):
    BOTH = "both"
    LEFT = "left"
    RIGHT = "right"

    def __str__(self) -> str:
        return str(self.value)

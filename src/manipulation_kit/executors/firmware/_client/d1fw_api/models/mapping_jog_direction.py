from enum import Enum


class MappingJogDirection(str, Enum):
    BACKWARD = "backward"
    FORWARD = "forward"
    LEFT = "left"
    RIGHT = "right"

    def __str__(self) -> str:
        return str(self.value)

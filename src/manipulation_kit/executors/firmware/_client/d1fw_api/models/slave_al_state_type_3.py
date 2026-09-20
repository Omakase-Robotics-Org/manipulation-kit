from enum import Enum


class SlaveAlStateType3(str, Enum):
    OP = "Op"

    def __str__(self) -> str:
        return str(self.value)

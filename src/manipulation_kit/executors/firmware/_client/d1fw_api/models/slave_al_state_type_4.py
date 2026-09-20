from enum import Enum


class SlaveAlStateType4(str, Enum):
    BOOTSTRAP = "Bootstrap"

    def __str__(self) -> str:
        return str(self.value)

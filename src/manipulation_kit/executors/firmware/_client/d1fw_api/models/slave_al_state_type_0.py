from enum import Enum


class SlaveAlStateType0(str, Enum):
    INIT = "Init"

    def __str__(self) -> str:
        return str(self.value)

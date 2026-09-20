from enum import Enum


class SlaveAlStateType1(str, Enum):
    PREOP = "PreOp"

    def __str__(self) -> str:
        return str(self.value)

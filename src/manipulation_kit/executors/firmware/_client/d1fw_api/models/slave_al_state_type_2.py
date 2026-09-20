from enum import Enum


class SlaveAlStateType2(str, Enum):
    SAFEOP = "SafeOp"

    def __str__(self) -> str:
        return str(self.value)

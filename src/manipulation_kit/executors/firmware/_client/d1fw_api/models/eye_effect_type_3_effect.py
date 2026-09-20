from enum import Enum


class EyeEffectType3Effect(str, Enum):
    CLEAR = "clear"

    def __str__(self) -> str:
        return str(self.value)

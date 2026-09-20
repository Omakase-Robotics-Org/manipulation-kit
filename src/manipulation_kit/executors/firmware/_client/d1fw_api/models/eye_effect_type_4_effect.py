from enum import Enum


class EyeEffectType4Effect(str, Enum):
    STOP = "stop"

    def __str__(self) -> str:
        return str(self.value)

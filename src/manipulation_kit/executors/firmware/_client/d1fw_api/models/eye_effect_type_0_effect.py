from enum import Enum


class EyeEffectType0Effect(str, Enum):
    BREATHING = "breathing"

    def __str__(self) -> str:
        return str(self.value)

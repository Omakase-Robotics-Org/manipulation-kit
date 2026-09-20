from enum import Enum


class EyeEffectType2Effect(str, Enum):
    SOLID = "solid"

    def __str__(self) -> str:
        return str(self.value)

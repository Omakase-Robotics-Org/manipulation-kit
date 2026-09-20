from enum import Enum


class EyeEffectType1Effect(str, Enum):
    RUNNING = "running"

    def __str__(self) -> str:
        return str(self.value)

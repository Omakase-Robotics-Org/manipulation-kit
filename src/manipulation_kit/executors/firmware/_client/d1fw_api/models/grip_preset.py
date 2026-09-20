from enum import Enum


class GripPreset(str, Enum):
    FIRM = "firm"
    SOFT = "soft"
    STRONG = "strong"

    def __str__(self) -> str:
        return str(self.value)

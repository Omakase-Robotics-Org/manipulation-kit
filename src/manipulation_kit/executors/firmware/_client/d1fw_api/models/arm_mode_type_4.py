from enum import Enum


class ArmModeType4(str, Enum):
    RELEASE = "release"

    def __str__(self) -> str:
        return str(self.value)

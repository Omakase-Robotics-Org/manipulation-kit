from enum import Enum


class ChassisParamType1Kind(str, Enum):
    LOW_OR_STRONG = "low_or_strong"

    def __str__(self) -> str:
        return str(self.value)

from enum import Enum


class LeaseClass(str, Enum):
    AMBIENT = "ambient"
    OPERATOR = "operator"
    POLICY = "policy"

    def __str__(self) -> str:
        return str(self.value)

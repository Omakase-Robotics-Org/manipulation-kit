from enum import Enum


class ArmModeRequestType3Mode(str, Enum):
    CARTESIAN_IMPEDANCE = "cartesian_impedance"

    def __str__(self) -> str:
        return str(self.value)

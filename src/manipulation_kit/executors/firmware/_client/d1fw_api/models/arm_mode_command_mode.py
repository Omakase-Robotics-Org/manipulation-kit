from enum import Enum


class ArmModeCommandMode(str, Enum):
    CARTESIAN_IMPEDANCE = "cartesian_impedance"
    FORCE_COMPLIANCE = "force_compliance"
    IDLE = "idle"
    POSITION = "position"
    TORQUE = "torque"

    def __str__(self) -> str:
        return str(self.value)

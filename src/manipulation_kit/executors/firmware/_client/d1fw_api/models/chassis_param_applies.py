from enum import Enum


class ChassisParamApplies(str, Enum):
    CHASSIS_DRIVER_RESTART = "chassis_driver_restart"
    DOCKING_RESTART = "docking_restart"
    NAVIGATION_RESTART = "navigation_restart"
    NO_KNOWN_READER = "no_known_reader"
    UNKNOWN = "unknown"

    def __str__(self) -> str:
        return str(self.value)

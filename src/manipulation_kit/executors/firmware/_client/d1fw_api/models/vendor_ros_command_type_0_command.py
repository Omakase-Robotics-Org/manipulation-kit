from enum import Enum


class VendorRosCommandType0Command(str, Enum):
    POSE_RESET = "pose_reset"

    def __str__(self) -> str:
        return str(self.value)

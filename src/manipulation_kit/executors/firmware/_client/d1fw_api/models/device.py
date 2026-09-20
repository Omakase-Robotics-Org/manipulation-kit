from enum import Enum


class Device(str, Enum):
    ARM = "arm"
    CHASSIS = "chassis"
    EYES = "eyes"
    GRIPPER = "gripper"
    HAND = "hand"
    NECK = "neck"
    SLIDER = "slider"

    def __str__(self) -> str:
        return str(self.value)

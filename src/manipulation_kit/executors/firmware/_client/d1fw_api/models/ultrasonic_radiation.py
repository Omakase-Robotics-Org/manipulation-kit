from enum import Enum


class UltrasonicRadiation(str, Enum):
    INFRARED = "infrared"
    ULTRASONIC = "ultrasonic"
    UNKNOWN = "unknown"

    def __str__(self) -> str:
        return str(self.value)

from enum import Enum


class ZeroVerification(str, Enum):
    PENDING = "pending"
    UNRECORDED = "unrecorded"
    VERIFIED = "verified"

    def __str__(self) -> str:
        return str(self.value)

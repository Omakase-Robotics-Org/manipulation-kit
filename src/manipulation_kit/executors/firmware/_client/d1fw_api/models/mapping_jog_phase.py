from enum import Enum


class MappingJogPhase(str, Enum):
    ACTIVE = "active"
    ENDED = "ended"
    IDLE = "idle"
    STOPPING = "stopping"

    def __str__(self) -> str:
        return str(self.value)

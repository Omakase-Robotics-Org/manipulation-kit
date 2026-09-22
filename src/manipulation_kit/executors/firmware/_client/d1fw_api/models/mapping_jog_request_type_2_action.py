from enum import Enum


class MappingJogRequestType2Action(str, Enum):
    STOP = "stop"

    def __str__(self) -> str:
        return str(self.value)

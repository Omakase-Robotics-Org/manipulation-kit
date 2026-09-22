from enum import Enum


class MappingJogRequestType0Action(str, Enum):
    START = "start"

    def __str__(self) -> str:
        return str(self.value)

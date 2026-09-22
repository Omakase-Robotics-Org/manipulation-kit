from enum import Enum


class MappingJogRequestType1Action(str, Enum):
    RENEW = "renew"

    def __str__(self) -> str:
        return str(self.value)

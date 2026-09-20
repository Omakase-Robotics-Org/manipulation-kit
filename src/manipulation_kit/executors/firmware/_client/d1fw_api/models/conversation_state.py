from enum import Enum


class ConversationState(str, Enum):
    CONVERSING = "conversing"
    STANDBY = "standby"
    STARTING = "starting"

    def __str__(self) -> str:
        return str(self.value)

from enum import Enum


class HandModel(str, Enum):
    LEADSHINEDH116S = "leadshine/dh116s"
    LINKERBOTO30 = "linkerbot/o30"

    def __str__(self) -> str:
        return str(self.value)

from enum import Enum


class MappingStreamStatus(str, Enum):
    ERROR = "error"
    FRESH = "fresh"
    NEVER_RECEIVED = "never_received"
    NOT_PUBLISHED_IN_THIS_MODE = "not_published_in_this_mode"
    STALE = "stale"

    def __str__(self) -> str:
        return str(self.value)

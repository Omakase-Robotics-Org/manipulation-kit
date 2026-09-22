from enum import Enum


class MappingJogDelivery(str, Enum):
    ACKNOWLEDGED = "acknowledged"
    NOT_ATTEMPTED = "not_attempted"
    OUTCOME_UNKNOWN = "outcome_unknown"
    REFUSED = "refused"

    def __str__(self) -> str:
        return str(self.value)
